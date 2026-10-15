"""Synchronous remote inference. Only this module knows about WebSockets."""
import logging
import os
import shlex
import time
from urllib.parse import parse_qsl, quote, urlsplit, urlunsplit
import numpy as np
from . import codec
from .calibration import STATE_DIM
from .settings import local_path

logger = logging.getLogger(__name__)


def read_env_file(name):
    """Read local KEY=value entries without executing shell code or exporting secrets."""
    if not name:
        return {}
    path = local_path(name)
    if not path.is_file():
        raise ValueError("The selected env file does not exist inside custom_control")
    values = {}
    for number, line in enumerate(path.read_text().splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].lstrip()
        if "=" not in line:
            raise ValueError(f"Env file line {number} must use KEY=value")
        key, value = line.split("=", 1)
        try:
            parts = shlex.split(value.strip(), comments=True, posix=True)
        except ValueError:
            raise ValueError(f"Invalid quoting on env file line {number}") from None
        if len(parts) > 1:
            raise ValueError(f"Quote values containing spaces on env file line {number}")
        values[key.strip()] = parts[0] if parts else ""
    return values


def _environment_value(name, env_file=None):
    """Read a named environment variable without putting its value in errors."""
    if not isinstance(name, str) or not name or "=" in name or "\x00" in name:
        raise ValueError("Environment variable names must be nonempty and contain no '=' or NUL")
    value = os.environ.get(name) or read_env_file(env_file).get(name)
    if not value:
        raise ValueError(f"Set {name} in the terminal or selected env file before starting inference")
    return value


def validate_server_url(url):
    """Require a token-free WebSocket URL suitable for configuration snapshots."""
    try:
        parsed = urlsplit(url)
        valid = (parsed.scheme in ("ws", "wss") and parsed.hostname
                 and not parsed.username and not parsed.password and not parsed.fragment)
        parsed.port  # Validate an optional port without echoing the URL.
        has_token = any(key.lower() == "token" for key, _ in parse_qsl(parsed.query, keep_blank_values=True))
    except (ValueError, TypeError):
        raise ValueError("Invalid server URL; use ws:// or wss:// with a valid host/port") from None
    if not valid:
        raise ValueError("Use a ws:// or wss:// server URL without user credentials or a fragment")
    if has_token:
        raise ValueError("Keep token out of server_url; use --token-env instead")
    return url


def connection_settings(cfg, *, server_url=None, server_url_env=None, token_env=None, env_file=None):
    """CLI URL > named URL environment variable > saved URL. Never store a token."""
    result = dict(cfg)
    file_name = env_file if env_file is not None else cfg.get("env_file")
    read_env_file(file_name)  # Validate file/quoting before opening hardware.
    url_name = server_url_env if server_url_env is not None else cfg.get("server_url_env")
    token_name = token_env if token_env is not None else cfg.get("token_env")
    if file_name and token_name is None:
        token_name = "JUPYTER_TOKEN"  # Same default token name as inference_example.py.
    url = server_url if server_url is not None else (_environment_value(url_name, file_name) if url_name else cfg["server_url"])
    # Clear the ignored URL-env selector for an explicit CLI URL so a later
    # PolicyClient construction cannot override that explicit choice again.
    result.update(server_url=validate_server_url(url),
                  server_url_env=None if server_url is not None else url_name, token_env=token_name, env_file=file_name)
    return result


def authenticated_url(server_url, token_env=None, env_file=None):
    """Add Jupyter's URL-encoded token only when making the actual connection."""
    validate_server_url(server_url)
    if not token_env:
        return server_url  # Direct policy servers can remain unauthenticated.
    token = _environment_value(token_env, env_file)
    parsed = urlsplit(server_url)
    query = parsed.query + ("&" if parsed.query else "") + "token=" + quote(token, safe="")
    return urlunsplit(parsed._replace(query=query))


class PolicyResponseError(ValueError):
    """A safe-to-log explanation; never include arbitrary server text or URLs."""


class PolicyClient:
    def __init__(self, cfg):
        self.cfg = connection_settings(cfg)
        # Fail on a missing token before the caller opens any robot devices.
        authenticated_url(self.cfg["server_url"], self.cfg.get("token_env"), self.cfg.get("env_file"))
        self.connection = None
        self.metadata = None

    def check_connection(self):
        """Authenticate and read metadata without sending observations or moving anything."""
        return self._request(None)

    def connect(self):
        from websockets.sync.client import connect
        # Preserve the Jupyter /proxy/8000/ path and existing query parameters.
        # The authenticated URL is local to this call, never saved in cfg/logs.
        url = authenticated_url(self.cfg["server_url"], self.cfg.get("token_env"), self.cfg.get("env_file"))
        parsed = urlsplit(self.cfg["server_url"])
        # Omit all queries and credentials from connection-status messages.
        endpoint = urlunsplit((parsed.scheme, parsed.netloc, parsed.path, "", ""))
        logger.info("Connecting to %s (token authentication: %s)", endpoint,
                    "enabled" if self.cfg.get("token_env") else "disabled")
        self.connection = connect(url, compression=None,
                                  max_size=self.cfg["max_message_bytes"],
                                  open_timeout=self.cfg["inference_timeout_s"],
                                  ping_interval=20, ping_timeout=self.cfg["inference_timeout_s"])
        self.metadata = codec.unpack(self.connection.recv(timeout=self.cfg["inference_timeout_s"]))
        if not isinstance(self.metadata, dict):
            raise PolicyResponseError("Server metadata must be a dictionary")
        # Metadata is server-specific. Enforce the known dimensions if given.
        for key in ("state_dim", "action_dim"):
            if key in self.metadata and self.metadata[key] != STATE_DIM:
                raise PolicyResponseError(f"Server {key} differs from the configured 8-value left-arm layout")
        logger.info("Server metadata keys: %s", sorted(self.metadata))
        logger.info("Policy WebSocket connected; metadata received")

    def infer(self, obs):
        return self._request(obs)

    def _request(self, obs):
        from websockets.exceptions import ConnectionClosed
        last = None
        for attempt in range(self.cfg["max_attempts"]):
            try:
                if self.connection is None:
                    self.connect()
                if obs is None:
                    return self.metadata
                self.connection.send(codec.pack(obs))
                reply = self.connection.recv(timeout=self.cfg["inference_timeout_s"])
                if isinstance(reply, str):
                    raise PolicyResponseError("Policy server rejected request (text response); check its server-side logs")
                result = codec.unpack(reply)
                if not isinstance(result, dict):
                    raise PolicyResponseError("Policy reply must be a dictionary")
                return result
            except (OSError, TimeoutError, ConnectionClosed) as exc:
                last = type(exc).__name__
                self.close()  # Never reuse a connection after a failed request.
                if attempt + 1 < self.cfg["max_attempts"]:
                    logger.warning("Policy connection interrupted (%s); retrying attempt %s/%s",
                                   last, attempt + 2, self.cfg["max_attempts"])
                    time.sleep(min(2 ** attempt, 4))
            except PolicyResponseError as exc:
                self.close()
                raise exc from None
            except Exception as exc:
                self.close()
                # WebSocket exceptions may contain the full authenticated URL
                # or a proxy response that echoes it. Report type/status only.
                status = getattr(getattr(exc, "response", None), "status_code", None)
                detail = f", HTTP {status}" if isinstance(status, int) else ""
                raise RuntimeError(f"Policy connection failed ({type(exc).__name__}{detail}); check server URL, proxy and token") from None
            except BaseException:
                self.close()
                raise
        raise RuntimeError(f"Inference unavailable after retries ({last})") from None

    def close(self):
        if self.connection is not None:
            connection, self.connection = self.connection, None
            try:
                connection.close()
            except Exception:
                logger.warning("Error closing policy connection")


class MockPolicy:
    """Return the observed pose repeatedly; useful for a no-motion bring-up."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.pose = None

    def infer(self, obs):
        if self.pose is None:
            self.pose = np.array(obs[self.cfg["observation"]["state_key"]], copy=True)
        chunk = np.repeat(self.pose[None], self.cfg["chunk_steps"], axis=0)
        return {self.cfg["action_key"]: chunk}

    def close(self):
        pass
