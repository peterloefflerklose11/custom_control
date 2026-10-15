"""Read space/q without waiting for Enter; always restore terminal settings."""
import select
import sys
import termios
import tty


class Keyboard:
    def __enter__(self):
        self.saved = None
        if sys.stdin.isatty():
            self.saved = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def poll(self):
        if self.saved is not None and select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return ""

    def __exit__(self, *args):
        if self.saved is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.saved)
