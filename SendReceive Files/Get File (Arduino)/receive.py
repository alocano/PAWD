import serial
import time
import sys

SERIAL_PORT = 'COM17'
BAUD_RATE = 115200
TIMEOUT = 10
OUTPUT_FILENAME = "received_from_esp32.txt"
READY_MARKER = "!SYSTEM: Ready"

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT, rtscts=False, dsrdtr=False)
    ser.setDTR(False)
    ser.setRTS(False)
    time.sleep(2)
    ser.reset_input_buffer()

    print("Waiting for ESP32 to be ready...")
    while True:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            continue
        print(f"< {line}")
        if READY_MARKER in line:
            break

    ser.write(b"SEND_FILE\n")

    header = ser.readline().decode().strip()
    if not header.startswith("!FILE_SIZE:"):
        print(f"Unexpected response: {header}")
        sys.exit(1)

    file_size = int(header.split(":")[1])
    print(f"File size: {file_size} bytes")

    with open(OUTPUT_FILENAME, "wb") as f:
        received = 0
        while received < file_size:
            chunk = ser.read(min(512, file_size - received))
            if not chunk:
                break
            f.write(chunk)
            received += len(chunk)
            print(f"\rProgress: {received}/{file_size}", end='')

    print("\nFile received.")

    eof = ser.readline().decode().strip()
    if eof == "!EOF":
        print("Transfer verified.")
    else:
        print(f"Warning: Expected EOF, got '{eof}'")

    ser.close()

except Exception as e:
    print(f"Error: {e}")