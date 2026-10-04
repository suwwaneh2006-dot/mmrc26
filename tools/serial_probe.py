import sys
import time

import serial


def main():
    port = sys.argv[1]
    command = sys.argv[2] if len(sys.argv) > 2 else ""
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
    with serial.Serial(port, 115200, timeout=0.1) as s:
        s.dtr = False
        s.rts = False
        time.sleep(0.2)
        s.reset_input_buffer()
        if command:
            s.write((command + "\n").encode())
            s.flush()
        end = time.time() + seconds
        while time.time() < end:
            data = s.read(4096)
            if data:
                sys.stdout.write(data.decode(errors="replace"))
                sys.stdout.flush()


if __name__ == "__main__":
    main()