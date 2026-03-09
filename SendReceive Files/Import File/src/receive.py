import serial
import time
import sys

# To run: type "python receive.py" in terminal

# --- Configuration ---
SERIAL_PORT = 'COM12'  # Change this to your port (e.g., '/dev/ttyUSB0' on Linux)
BAUD_RATE = 115200
TIMEOUT = 10  # Seconds
OUTPUT_FILENAME = "received_from_esp32.txt"

# --- Main Script ---
try:
    # Open the serial port
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    time.sleep(2)  # Wait for the connection to stabilize
    print(f"Connected to {SERIAL_PORT}. Sending command...")

    # Send the command to the ESP32
    ser.write(b"SEND_FILE\n")

    # --- Read the header (FILE_SIZE) ---
    header_line = ser.readline().decode().strip()
    if not header_line.startswith("!FILE_SIZE:"):
        print(f"Unexpected response: {header_line}")
        ser.close()
        sys.exit(1)

    file_size = int(header_line.split(":")[1])
    print(f"File size reported: {file_size} bytes. Receiving data...")

    # --- Read the raw file data ---
    bytes_received = 0
    with open(OUTPUT_FILENAME, "wb") as output_file:
        while bytes_received < file_size:
            # Read in chunks, but not more than what's left
            chunk_size = min(512, file_size - bytes_received)
            data_chunk = ser.read(chunk_size)

            if not data_chunk:
                print("\nError: Timeout or no data received.")
                break

            output_file.write(data_chunk)
            bytes_received += len(data_chunk)
            # Simple progress indicator
            print(f"\rProgress: {bytes_received}/{file_size} bytes", end='')

    print("\nFile data received.")

    # --- Verify the End-of-File marker ---
    eof_line = ser.readline().decode().strip()
    if eof_line == "!EOF":
        print("File transfer verified successfully.")
    else:
        print(f"Warning: Expected EOF marker, got: {eof_line}")

    ser.close()

except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    print("Please check the port name and that it's not in use by another program.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")