import serial
import time
import sys

# ===== Configuration =====
SERIAL_PORT = 'COM9'           # Change to your ESP32's port
BAUD_RATE = 115200
TIMEOUT = 10                    # Seconds for serial reads
OUTPUT_FILENAME = "received_from_esp32.txt"
READY_MARKER = "!SYSTEM: Ready" # The exact string the ESP32 prints when ready

# ===== Main Script =====
try:
    # Open the serial port with hardware flow control disabled
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=TIMEOUT,
        rtscts=False,
        dsrdtr=False
    )

    # Immediately de-assert DTR and RTS so the ESP32 is not held in reset
    ser.setDTR(False)
    ser.setRTS(False)

    # Give the ESP32 time to boot up and print its startup messages
    time.sleep(3)

    # Clear any stale data that might have arrived during boot
    ser.reset_input_buffer()

    print(f"Waiting for ESP32 to be ready (looking for '{READY_MARKER}')...")

    # Read lines until we see the ready marker
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
        except UnicodeDecodeError:
            # If we get garbage bytes, just skip them
            continue

        if not line:
            # Timeout – no data received, but we keep waiting
            continue

        print(f"< {line}")   # Optional: show each line received

        if READY_MARKER in line:
            print("ESP32 is ready. Sending command...")
            break

    # Send the command to start file transfer
    ser.write(b"SEND_FILE\n")

    # --- Read the file size header ---
    header_line = ser.readline().decode('utf-8').strip()
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
            print(f"\rProgress: {bytes_received}/{file_size} bytes", end='')

    print("\nFile data received.")

    # --- Verify the End-of-File marker ---
    eof_line = ser.readline().decode('utf-8').strip()
    if eof_line == "!EOF":
        print("File transfer verified successfully.")
    else:
        print(f"Warning: Expected EOF marker, got: '{eof_line}'")

    ser.close()
    print(f"File saved as '{OUTPUT_FILENAME}'")

except serial.SerialException as e:
    print(f"Error opening or using serial port: {e}")
    print("Please check that:")
    print(f"  - The port '{SERIAL_PORT}' is correct")
    print("  - No other program (like the PlatformIO Serial Monitor) is using it")
    print("  - The ESP32 is connected and powered")
except Exception as e:
    print(f"An unexpected error occurred: {e}")