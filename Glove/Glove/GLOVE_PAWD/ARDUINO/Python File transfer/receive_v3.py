
# how to run: python receive_v3.py --port COM11 --baud 115200 --duration 30 --outfile mydata.txt
# receive_v3.py -- with filter, no dropped file
import argparse
import time
import re
import serial

KEEP_PATTERN = re.compile(r"\[(SAMPLE|ROT|TAP)\]")

def main():
    print(">>> RUNNING receive_v3.py (with filter) <<<")

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--outfile", default="serial_log.txt")
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser, \
         open(args.outfile, "w", encoding="utf-8", errors="replace") as f:

        time.sleep(0.5)
        ser.reset_input_buffer()

        print(f"Logging to {args.outfile} for {args.duration}s")
        start = time.time()
        kept = 0
        dropped = 0

        while time.time() - start < args.duration:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            if KEEP_PATTERN.search(line):
                f.write(line + "\n")
                f.flush()
                kept += 1
                print(f"KEEP [{kept}]: {line}")
            else:
                dropped += 1   # just a counter, not written to disk

        print(f"Done. Kept {kept}, dropped {dropped}.")

if __name__ == "__main__":
    main()