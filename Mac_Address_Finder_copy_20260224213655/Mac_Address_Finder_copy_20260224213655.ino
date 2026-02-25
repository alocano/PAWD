#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>

// --- OLED Configuration for 128x64 (0.96" Monochrome Display) ---
#define SCREEN_WIDTH 128 // Display width
#define SCREEN_HEIGHT 64 // Display height
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)

// Initialize the display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);


//if using screen with pins change 0x3D to 0X3C
// if using the super mini i2c pins are either 8 and 9 if no response flip the pins
// if using wroom i2c pins 23 and 18 if no response flip the pins
  // 1. Initialize the OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { 
    Serial.println(F("SSD1306 allocation failed. Check wiring or I2C address (0x3C or 0x3D)."));
    while(true); // Halt execution if display fails
  }

  // Clear the display buffer and set text properties
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); // Draw white text

  // 2. Get the MAC Address using the standard WiFi.macAddress() function
  
  // Set to Wi-Fi Station mode for MAC address initialization
  WiFi.mode(WIFI_MODE_STA); 
  delay(500); // Small delay to ensure initialization is complete

  // Get the MAC Address string directly
  String macStr = WiFi.macAddress(); 
  
  // 3. Display the MAC Address on the OLED
  
  // Title Text (Size 1: smallest font)
  display.setTextSize(1);
  display.setCursor(0, 0); 
  display.println("--- ESP32 MAC Address ---");
  
  // Display the address string directly
  display.setCursor(0, 15);
  display.println(macStr.substring(0, 9)); // Print first half (XX:XX:XX:X)
  
  display.setCursor(0, 25);
  display.println(macStr.substring(9));    // Print second half (X:XX:XX:XX)
  
  // Instructions Text
  display.setCursor(0, 45);
  display.println("Copy this address for");
  display.println("your Sender sketch.");

  display.display(); // Push the buffer content to the screen

  // 4. Print to Serial Monitor as backup
  Serial.print("OLED is displaying this MAC: ");
  Serial.println(macStr);
}

void loop() {
  // The address is displayed once in setup, so the loop is empty.
}