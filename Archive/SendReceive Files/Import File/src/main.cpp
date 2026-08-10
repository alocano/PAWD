/* 
Make sure COM # and Baud rate in ini match Device Manager serial COM port info!

Step 1. Build and upload only the filesystem (contents of /data folder on computer)
    Terminal:pio run --target -uploadfs
    or use sidebar:
    PlatformIO -> Project Tasks -> esp32dev -> Platform -> Upload Filesystem Image
Step 2. Upload main.cpp code
    Terminal: pio run --target upload
    or use sidebar:
    PlatformIO -> Project Tasks -> esp32dev -> General -> Upload
Step 3. Verification
    Open PlatformIO serial monitor (plug icon on bottom toolbar)
    Type SEND_FILE and press enter to send command
        If having trouble sending command, try using a serial terminal like PuTTY
        with the correct COM port and baud rate (115200)
        OR just type/spam the keystrokes ctrl+t and ctrl+r and maybe even tap the reset button
        on the ESP32 to get the system to recognize the command input, THEN type SEND_FILE
            Also if SEND_FILE still doesn't work, immediately type and enter it again
    Should see file size header, file content, then end of file
Step 4. Run python code (while in /src folder)
    Terminal: python receive.py
    Press EN/reset button on ESP32 to trigger file transfer
    Should see file content printed in terminal, and file "received_from_esp32.txt" appear in /src folder
*/
#include <Arduino.h>
#include "SPIFFS.h"

const String CMD_SEND_FILE = "SEND_FILE";

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }

    Serial.println("\n\n--- ESP32 File Transfer ---");

    // Mount SPIFFS, format if necessary
    if (!SPIFFS.begin(true)) {
        Serial.println("!ERROR: SPIFFS Mount Failed");
        return;
    }

    // List all files in the root (helpful for verification)
    File root = SPIFFS.open("/");
    if (root) {
        File file = root.openNextFile();
        while (file) {
            Serial.print("FILE: ");
            Serial.print(file.name());
            Serial.print(" (");
            Serial.print(file.size());
            Serial.println(" bytes)");
            file = root.openNextFile();
        }
    }

    // Signal that the system is ready for commands
    Serial.println("!SYSTEM: Ready. Send SEND_FILE to start transfer.");
}

// The file transfer function – only outputs protocol data
void sendFile(String path) {
    File file = SPIFFS.open(path, FILE_READ);
    if (!file) {
        Serial.println("!ERROR: File not found");
        return;
    }

    // 1. Send file size header
    Serial.print("!FILE_SIZE:");
    Serial.println(file.size());
    delay(50);   // Give the PC a moment to switch to data reading mode

    // 2. Send raw file data in chunks
    uint8_t buffer[128];
    size_t bytesRead;
    while ((bytesRead = file.read(buffer, sizeof(buffer))) > 0) {
        Serial.write(buffer, bytesRead);
        delay(5);  // Small delay to prevent overwhelming the USB buffer
    }
    file.close();

    // 3. Send end-of-file marker
    Serial.println("!EOF");
}

void loop() {
    if (Serial.available()) {
        String received = Serial.readStringUntil('\n');
        received.trim();

        if (received.equals(CMD_SEND_FILE)) {
            sendFile("/data.txt");
        }
    }
}