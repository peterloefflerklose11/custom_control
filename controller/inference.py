"""One inference request in flight; only this thread touches the policy client."""
import queue
import threading


class InferenceWorker:
    def __init__(self, client):
        self.client = client
        self.requests = queue.Queue(maxsize=1)
        self.results = queue.Queue()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, token, observation):
        self.requests.put_nowait((token, observation))

    def poll(self):
        try:
            return self.results.get_nowait()
        except queue.Empty:
            return None

    def _run(self):
        try:
            while not self.stop.is_set():
                try:
                    token, observation = self.requests.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    reply, error = self.client.infer(observation), None
                except Exception as exc:
                    reply, error = None, exc
                self.results.put((token, reply, error))
        finally:
            self.client.close()

    def close(self):
        self.stop.set()
        # A blocked network call is bounded by its timeout. The daemon owns
        # the connection until it returns; don't race close() with recv().
        self.thread.join(timeout=0.25)
