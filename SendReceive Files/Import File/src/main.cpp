#include <Arduino.h>
#include "SPIFFS.h"

/* 
Step 1. Build and upload only the filesystem (contents of /data folder on computer)
    Terminal:pio run --target -uploadfs
    or use sidebar:
    PlatformIO -> Project Tasks -> esp32dev -> Platform -> Upload Filesystem Image
Step 2. Upload main.cpp code
    Terminal: pio run --target upload
    or use sidebar:
    PlatformIO -> Project Tasks -> esp32dev -> General -> Upload
Verification
    Open PlatformIO serial monitor (plug icon on bottom toolbar)
    Type SEND_FILE and press enter to send command
    Should see file size header, file content, then end of file
*/

const String CMD_SEND_FILE = "SEND_FILE";

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }

    if (!SPIFFS.begin(true)) {
        Serial.println("!ERROR: SPIFFS Mount Failed");
        return;
    }
    Serial.println("!SYSTEM: SPIFFS Ready. Awaiting command...");
}



void sendFile(String path) {
    File file = SPIFFS.open(path, FILE_READ);
    if (!file) {
        Serial.println("!ERROR: File not found");
        return;
    }

    size_t fileSize = file.size();
    Serial.print("!FILE_SIZE:");
    Serial.println(fileSize);
    delay(50);

    uint8_t buffer[128];
    size_t bytesRead;
    while ((bytesRead = file.read(buffer, sizeof(buffer))) > 0) {
        Serial.write(buffer, bytesRead);
        delay(5);
    }
    file.close();

    Serial.println("!EOF");
    Serial.println("!TRANSMISSION: Complete");
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.equals(CMD_SEND_FILE)) {
            sendFile("/data.txt");
        }
    }
}