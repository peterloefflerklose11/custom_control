"""Real loopback WebSocket test with an independent openpi-style encoder."""
import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import msgpack
import numpy as np
from websockets.sync.server import serve
from controller import codec
from controller.settings import load
from controller.transport import PolicyClient


class NetworkTests(unittest.TestCase):
    def test_numeric_wire_roundtrip(self):
        values = {"state": np.arange(8, dtype=np.float32), "task": "pick", "flag": np.bool_(True)}
        decoded = codec.unpack(codec.pack(values))
        np.testing.assert_array_equal(values["state"], decoded["state"])
        self.assertEqual(decoded["task"], "pick")
        self.assertTrue(decoded["flag"])
        with self.assertRaises(ValueError):
            codec.pack({"bad": np.array([object()], dtype=object)})

    def test_openpi_compatible_handshake_and_inference(self):
        received = []
        def handler(connection):
            connection.send(msgpack.packb({"state_dim": 8, "action_dim": 8}))
            obs = msgpack.unpackb(connection.recv(), strict_map_key=False)
            received.append(obs)
            state = obs["observation.state"]
            # Deliberately encode using the server's documented representation,
            # without using the client's codec implementation.
            actions = np.frombuffer(state[b"data"], dtype=state[b"dtype"]).reshape(1, 8)
            connection.send(msgpack.packb({"actions": {b"__ndarray__": True,
                b"data": actions.tobytes(), b"dtype": actions.dtype.str, b"shape": actions.shape}}))
        # Native top-view RGB alone is 1.44 MB, above the library's default
        # 1 MiB receive limit. Match the deployed policy-server frame budget.
        with serve(handler, "127.0.0.1", 0, max_size=32 * 1024 * 1024) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            cfg = load()
            cfg["server_url"] = f"ws://127.0.0.1:{server.socket.getsockname()[1]}"
            client = PolicyClient(cfg)
            try:
                from controller.observation import build
                from controller.cameras import MockCameras
                state = np.arange(8, dtype=np.float32)
                obs = build(state, MockCameras(cfg).frames(), "hold", cfg)
                result = client.infer(obs)
                np.testing.assert_array_equal(result["actions"], state[None])
                self.assertEqual(client.metadata["state_dim"], 8)
                self.assertEqual(received[0]["task"], "hold")
                self.assertEqual(set(received[0]), {"observation.state", "task", "observation.images.overhead",
                                                   "observation.images.wrist"})
                self.assertEqual(received[0]["observation.images.wrist"][b"shape"], [360, 640, 3])
            finally:
                client.close()
                server.shutdown()
                thread.join(timeout=2)

    def test_text_rejection_is_fatal(self):
        calls = []
        def handler(connection):
            calls.append(1)
            connection.send(msgpack.packb({}))
            connection.recv()
            connection.send("schema rejected")
        with serve(handler, "127.0.0.1", 0) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            cfg = load()
            cfg["server_url"] = f"ws://127.0.0.1:{server.socket.getsockname()[1]}"
            client = PolicyClient(cfg)
            try:
                with self.assertRaisesRegex(ValueError, "rejected request"):
                    client.infer({"task": "test"})
                self.assertEqual(len(calls), 1)
            finally:
                client.close()
                server.shutdown()
                thread.join(timeout=2)

    def test_bimanual_metadata_rejected_before_observation_send(self):
        class Connection:
            closed = False
            def recv(inner, timeout):
                return codec.pack({"state_dim": 16, "action_dim": 16})
            def send(inner, data):
                self.fail("Observation sent to incompatible bimanual server")
            def close(inner):
                inner.closed = True
        connection = Connection()
        client = PolicyClient(load())
        with patch("websockets.sync.client.connect", return_value=connection):
            with self.assertRaisesRegex(ValueError, "8-value left-arm"):
                client.infer({"observation.state": np.zeros(8)})
        self.assertTrue(connection.closed)


if __name__ == "__main__":
    unittest.main()
