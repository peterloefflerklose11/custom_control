"""Jupyter token handling using fake credentials and loopback connections only."""
import contextlib
import io
import json
import os
from pathlib import Path
import sys
import threading
import tempfile
import ast
import unittest
from unittest.mock import patch
from urllib.parse import parse_qs, quote, urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import msgpack
from websockets.sync.server import serve
from controller.settings import load
from controller.transport import authenticated_url, connection_settings, read_env_file, PolicyClient
from controller.settings import ROOT
from run import main


class AuthenticationTests(unittest.TestCase):
    def test_authenticated_url_matches_working_example(self):
        # Extract just the helper: do not import or run the SO-101 script.
        source = ast.parse((ROOT / "inference_example.py").read_text())
        helper = next(n for n in source.body if isinstance(n, ast.FunctionDef) and n.name == "authenticated_url")
        namespace = {"os": os, "quote": quote}
        exec(compile(ast.Module(body=[helper], type_ignores=[]), "working_helper", "exec"), namespace)
        for base in ("wss://example.test/proxy/8000/", "wss://example.test/proxy/8000/?mode=test"):
            with patch.dict(os.environ, {"CONTROL_TEST_TOKEN": "fake token+&/?"}):
                expected = namespace["authenticated_url"](base, "CONTROL_TEST_TOKEN", {})
                self.assertEqual(authenticated_url(base, "CONTROL_TEST_TOKEN"), expected)

    def test_local_env_file_fallback_and_environment_precedence(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as temp:
            path = Path(temp) / "station.env"
            path.write_text("# Connection only\nexport JUPYTER_TOKEN='fake file token+&'\nPOLICY_SERVER_URL=wss://example.test/proxy/8000/\n")
            relative = str(path.relative_to(ROOT))
            with patch.dict(os.environ, {}, clear=True):
                cfg = connection_settings(load(), env_file=relative, server_url_env="POLICY_SERVER_URL")
                self.assertEqual(cfg["token_env"], "JUPYTER_TOKEN")
                client = PolicyClient(cfg)
                url = authenticated_url(cfg["server_url"], cfg["token_env"], cfg["env_file"])
                self.assertEqual(parse_qs(urlsplit(url).query)["token"], ["fake file token+&"])
                self.assertNotIn("fake file token", json.dumps(client.cfg))
            with patch.dict(os.environ, {"JUPYTER_TOKEN": "fake terminal token"}):
                url = authenticated_url(cfg["server_url"], cfg["token_env"], relative)
                self.assertEqual(parse_qs(urlsplit(url).query)["token"], ["fake terminal token"])

    def test_env_file_must_stay_inside_folder(self):
        with self.assertRaises(ValueError):
            read_env_file("../station.env")

    def test_cli_connection_check_never_constructs_hardware(self):
        output = io.StringIO()
        with patch("controller.transport.PolicyClient") as client, \
             patch("controller.hardware.Robot") as robot, \
             patch("controller.cameras.Cameras") as cameras, \
             patch("controller.recording.Recorder") as recorder, \
             contextlib.redirect_stdout(output):
            self.assertEqual(main(["--mode", "hardware", "--check-connection"]), 0)
            client.return_value.check_connection.assert_called_once()
            client.return_value.close.assert_called_once()
            robot.assert_not_called()
            cameras.assert_not_called()
            recorder.assert_not_called()
        self.assertIn("No robot/cameras opened", output.getvalue())

    def test_connection_check_performs_handshake_without_inference(self):
        from websockets.exceptions import ConnectionClosed
        received = []
        def handler(connection):
            connection.send(msgpack.packb({"state_dim": 8, "action_dim": 8}))
            try:
                received.append(connection.recv(timeout=2))
            except ConnectionClosed:
                pass
        with serve(handler, "127.0.0.1", 0) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            cfg = connection_settings(load(), server_url=f"ws://127.0.0.1:{server.socket.getsockname()[1]}/proxy/8000/")
            client = PolicyClient(cfg)
            try:
                self.assertEqual(client.check_connection()["state_dim"], 8)
            finally:
                client.close()
                server.shutdown()
                thread.join(timeout=2)
            self.assertEqual(received, [])

    def test_no_auth_leaves_direct_url_unchanged(self):
        url = "ws://localhost:8000/path/?mode=test"
        self.assertEqual(authenticated_url(url), url)

    def test_jupyter_path_existing_query_and_token_encoding(self):
        token = "fake token+/&=?#ü"
        for base in ("wss://example.test/proxy/8000/", "wss://example.test/proxy/8000/?mode=a%2Fb"):
            with patch.dict(os.environ, {"CONTROL_TEST_TOKEN": token}):
                result = authenticated_url(base, "CONTROL_TEST_TOKEN")
            parsed = urlsplit(result)
            self.assertEqual(parsed.path, "/proxy/8000/")
            self.assertEqual(parse_qs(parsed.query)["token"], [token])
            self.assertIn("token=" + quote(token, safe=""), result)
            if "?" in base:
                self.assertEqual(parse_qs(parsed.query)["mode"], ["a/b"])

    def test_missing_token_is_clear_before_connection(self):
        cfg = load()
        cfg["token_env"] = "CONTROL_TEST_TOKEN"
        with patch.dict(os.environ, {}, clear=True), patch("websockets.sync.client.connect") as connect:
            with self.assertRaisesRegex(ValueError, "Set CONTROL_TEST_TOKEN"):
                PolicyClient(cfg)
            connect.assert_not_called()

    def test_url_environment_and_cli_precedence(self):
        cfg = load()
        with patch.dict(os.environ, {"CONTROL_TEST_URL": "wss://example.test/proxy/8000/"}):
            effective = connection_settings(cfg, server_url_env="CONTROL_TEST_URL", token_env="CONTROL_TEST_TOKEN")
            self.assertEqual(effective["server_url"], os.environ["CONTROL_TEST_URL"])
            cfg["server_url_env"] = "CONTROL_TEST_URL"
            direct = connection_settings(cfg, server_url="ws://localhost:9000/")
            self.assertEqual(connection_settings(direct)["server_url"], "ws://localhost:9000/")
            self.assertIsNone(direct["server_url_env"])

    def test_missing_url_variable_does_not_fall_back_silently(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(ValueError, "Set CONTROL_TEST_URL"):
                connection_settings(load(), server_url_env="CONTROL_TEST_URL")

    def test_credentials_are_not_allowed_inside_saved_url(self):
        for url in ("wss://example.test/proxy/8000/?token=fake-secret",
                    "wss://user:fake-secret@example.test/proxy/8000/"):
            with self.assertRaises(ValueError) as error:
                connection_settings(load(), server_url=url)
            self.assertNotIn("fake-secret", str(error.exception))

    def test_cli_accepts_environment_variable_names(self):
        args = ["--check-config", "--policy", "websocket", "--server-url-env", "CONTROL_TEST_URL",
                "--token-env", "CONTROL_TEST_TOKEN"]
        env = {"CONTROL_TEST_URL": "wss://example.test/proxy/8000/", "CONTROL_TEST_TOKEN": "fake-secret"}
        output = io.StringIO()
        with patch.dict(os.environ, env), contextlib.redirect_stdout(output):
            self.assertEqual(main(args), 0)
        self.assertNotIn("fake-secret", output.getvalue())

    def test_effective_config_never_contains_resolved_token(self):
        with patch.dict(os.environ, {"CONTROL_TEST_TOKEN": "fake-secret"}):
            cfg = connection_settings(load(), token_env="CONTROL_TEST_TOKEN")
            client = PolicyClient(cfg)
            serialized = json.dumps(client.cfg)
        self.assertNotIn("fake-secret", serialized)
        self.assertEqual(client.cfg["token_env"], "CONTROL_TEST_TOKEN")
        self.assertNotIn("token=", client.cfg["server_url"])

    def test_loopback_proxy_receives_encoded_token(self):
        requests = []
        def handler(connection):
            requests.append(connection.request.path)
            connection.send(msgpack.packb({"state_dim": 8, "action_dim": 8}))
            connection.recv()
            connection.send(msgpack.packb({"actions": [[0.0] * 8]}))
        with serve(handler, "127.0.0.1", 0) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            env = {"CONTROL_TEST_URL": f"ws://127.0.0.1:{server.socket.getsockname()[1]}/proxy/8000/?mode=test",
                   "CONTROL_TEST_TOKEN": "fake token&+/?"}
            with patch.dict(os.environ, env):
                cfg = connection_settings(load(), server_url_env="CONTROL_TEST_URL", token_env="CONTROL_TEST_TOKEN")
                client = PolicyClient(cfg)
                try:
                    result = client.infer({"task": "hold"})
                    self.assertEqual(len(result["actions"][0]), 8)
                    self.assertEqual(urlsplit(requests[0]).path, "/proxy/8000/")
                    self.assertEqual(parse_qs(urlsplit(requests[0]).query)["token"], [env["CONTROL_TEST_TOKEN"]])
                    self.assertEqual(parse_qs(urlsplit(requests[0]).query)["mode"], ["test"])
                finally:
                    client.close()
                    server.shutdown()
                    thread.join(timeout=2)

    def test_connection_failures_do_not_expose_authenticated_url(self):
        cfg = load()
        cfg.update(token_env="CONTROL_TEST_TOKEN", max_attempts=1)
        token = "fake token+/&"
        for error in (OSError("failed URL ?token=" + quote(token, safe="")),
                      RuntimeError("proxy echoed " + token)):
            with patch.dict(os.environ, {"CONTROL_TEST_TOKEN": token}), patch("websockets.sync.client.connect", side_effect=error):
                client = PolicyClient(cfg)
                with self.assertRaises(RuntimeError) as raised:
                    client.infer({})
                self.assertNotIn(token, str(raised.exception))
                self.assertNotIn(quote(token, safe=""), str(raised.exception))
                self.assertTrue(raised.exception.__suppress_context__)


if __name__ == "__main__":
    unittest.main()
