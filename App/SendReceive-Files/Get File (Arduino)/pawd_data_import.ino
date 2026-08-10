"""
receive.py  -  Serial bridge: ESP32 --> PC --> Flask

Usage:
    python receive.py --port COM17 --patient patient1

Arguments:
    --port      Serial port the ESP32 is connected to  (default: COM17)
    --patient   Patient ID to associate results with   (required)
    --no-clear  Skip clearing the file on the ESP32 after transfer

Run this once after each test session with Flask (app.py) already running.
"""

import argparse
import sys
import time
import requests
import serial

DEFAULT_PORT     = "COM17"
BAUD_RATE        = 115200
SERIAL_TIMEOUT   = 10
BOOT_WAIT        = 2
FLASK_INGEST_URL = "http://127.0.0.1:5000/api/ingest"
READY_MARKER     = "!SYSTEM: Ready"
OUTPUT_FILENAME  = "received_from_esp32.txt"


def parse_args():
    parser = argparse.ArgumentParser(description="Transfer data from ESP32 to Flask.")
    parser.add_argument("--port",     default=DEFAULT_PORT, help="Serial port (e.g. COM17)")
    parser.add_argument("--patient",  required=True,        help="Patient ID")
    parser.add_argument("--no-clear", action="store_true",  help="Do not clear file on ESP32 after transfer")
    return parser.parse_args()


def wait_for_ready(ser):
    print(f"Waiting for ESP32 ('{READY_MARKER}')...")
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception:
            continue
        if not line:
            continue
        print(f"  ESP32: {line}")
        if READY_MARKER in line:
            print("ESP32 is ready.")
            return


def receive_file(ser):
    ser.write(b"SEND_FILE\n")
    print("Sent: SEND_FILE")

    header = ser.readline().decode("utf-8", errors="ignore").strip()
    if not header.startswith("!FILE_SIZE:"):
        raise RuntimeError(f"Unexpected response: '{header}'")

    file_size = int(header.split(":")[1])
    print(f"File size: {file_size} bytes. Receiving...")

    received  = 0
    raw_bytes = bytearray()
    while received < file_size:
        chunk = ser.read(min(512, file_size - received))
        if not chunk:
            raise RuntimeError("Timeout before file transfer completed.")
        raw_bytes += chunk
        received  += len(chunk)
        print(f"\r  Progress: {received}/{file_size} bytes", end="", flush=True)
    print()

    eof = ser.readline().decode("utf-8", errors="ignore").strip()
    if eof != "!EOF":
        print(f"Warning: expected '!EOF', got '{eof}'")
    else:
        print("Transfer verified (!EOF received).")

    return raw_bytes.decode("utf-8", errors="ignore")


def clear_device_file(ser):
    ser.write(b"CLEAR_FILE\n")
    response = ser.readline().decode("utf-8", errors="ignore").strip()
    print("ESP32 file cleared." if "FILE DELETED" in response else f"Clear response: {response}")


def post_to_flask(patient_id, content):
    print(f"Posting to Flask ({FLASK_INGEST_URL})...")
    try:
        resp = requests.post(
            FLASK_INGEST_URL,
            json={"patient_id": patient_id, "content": content},
            timeout=10,
        )
        resp.raise_for_status()
        result = resp.json()
        print(f"Ingest complete: {result.get('inserted', 0)} row(s) inserted, "
              f"{result.get('skipped', 0)} line(s) skipped.")
    except requests.exceptions.ConnectionError:
        print("ERROR: Could not connect to Flask on port 5000.")
        print("  Make sure 'python backend/app.py' is running first.")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR posting to Flask: {e}")
        sys.exit(1)


def main():
    args = parse_args()
    print(f"Port: {args.port}  |  Patient: {args.patient}")

    try:
        ser = serial.Serial(
            port=args.port, baudrate=BAUD_RATE,
            timeout=SERIAL_TIMEOUT, rtscts=False, dsrdtr=False,
        )
        ser.setDTR(False)
        ser.setRTS(False)
        time.sleep(BOOT_WAIT)
        ser.reset_input_buffer()

        wait_for_ready(ser)
        content = receive_file(ser)

        with open(OUTPUT_FILENAME, "w", encoding="utf-8") as f:
            f.write(content)
        print(f"File saved locally as '{OUTPUT_FILENAME}'.")

        if not args.no_clear:
            clear_device_file(ser)

        ser.close()

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        print(f"  Check that '{args.port}' is correct and not used by another program.")
        sys.exit(1)
    except Exception as e:
        print(f"Error during transfer: {e}")
        sys.exit(1)

    post_to_flask(args.patient, content)
    print("Done.")


if __name__ == "__main__":
    main()