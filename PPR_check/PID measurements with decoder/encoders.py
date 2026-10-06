import os
import subprocess
import threading
import time

DECODER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "decoder")


class Encoders:
    def __init__(self, pins=(6, 5, 27, 17)):
        """pins = (left_1, left_2, right_1, right_2). Swap a wheel's two pins to flip its sign."""
        self.left = 0
        self.right = 0
        self.proc = subprocess.Popen([DECODER, *map(str, pins)],
                                     stdout=subprocess.PIPE, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            parts = line.split()
            if len(parts) == 2:          # skip anything that isn't "left right"
                self.left, self.right = int(parts[0]), int(parts[1])

    def close(self):
        self.proc.terminate()            # the decoder handles SIGTERM and releases the pins


if __name__ == "__main__":
    enc = Encoders()
    print("Turn the wheels by hand. Ctrl+C to stop.")
    try:
        while True:
            print(f"left = {enc.left:6d}   right = {enc.right:6d}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        enc.close()