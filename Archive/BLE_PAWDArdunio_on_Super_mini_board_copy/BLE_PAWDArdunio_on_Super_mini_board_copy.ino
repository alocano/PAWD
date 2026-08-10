#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 
#define SCREEN_ADDRESS 0x3D

#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define COUNT_CHARACTERISTIC_UUID "87654321-4321-4321-4321-ba0987654321"

// Official Pin 8 is the LED_PIN 
const int LED_PIN = 8; 
const int Sensor_pin = 0; //Pressure sensor pin
const int Rot_Pin = 1; // external buttons
const int Tap_Pin= 2; //external buttons

// Custom I2C pins to avoid the Pin 8 conflict
const int sda_pin = 10; 
const int scl_pin = 9;

int clk = 0; //for delay
int count = 0; // amount of taps
int count2=0;
int SensorData=0; //initalizing pressure sensor to zero 
bool pressed = false; //prevents overcounting in taps

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
enum {
  Start_State,
  Trans_State,
  Taps_State,
  Rot_State,
  End_State,
  Delay_State,
  Delay_State2
} currentState; // Variable to hold the current state

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// Callback to track connection status
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

void setup() {
  delay(1000);
  pinMode(LED_PIN, OUTPUT);
  pinMode(Sensor_pin,INPUT); //pressure sensor pin
  pinMode(Rot_Pin,INPUT_PULLUP); //setting pins to take input
  pinMode(Tap_Pin,INPUT_PULLUP);
  // Set the initial state when the program starts
  currentState =  Start_State; 
  Serial.println("FSM Starting in  Start_State");
  
  // Start I2C on Pin 10 (SDA) and Pin 9 (SCL)
  Wire.begin(sda_pin, scl_pin);

 /* if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // If it fails, the LED_PIN will blink fast to alert you
    while(1) {
      digitalWrite(LED_PIN, HIGH); delay(100);
      digitalWrite(LED_PIN, LOW);  delay(100);
    }
  }*/

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // If initialization fails, print error and stop
    Serial.begin(115200);//baud rate
    Serial.println(F("SSD1306 initialization failed"));
    for(;;); 
    
  }

  /*display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("C3 SuperMini");
  display.println("SDA: Pin 10");
  display.display();*/

  display.clearDisplay(); 
  display.setTextSize(2);         // Set text size to 2 (larger text)
  display.setTextColor(SSD1306_WHITE); // Set text color to white


  // 1. Give your device a name
  BLEDevice::init("Super-Mini-BLE");
  BLEDevice::setMTU(517);

  // 2. Create the server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());


  // Create Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create Characteristic with NOTIFY property
  pCharacteristic = pService->createCharacteristic(
                      COUNT_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Required for Notifications to work with most phone apps
  pCharacteristic->addDescriptor(new BLE2902());
  // Inside setup(), right after pCharacteristic->addDescriptor(new BLE2902());
 // BLEDescriptor *pDescriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  //pDescriptor->setValue("Button Counter Service");
  //pCharacteristic->addDescriptor(pDescriptor);

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection...");
  //Serial.print("My BLE Address is: ");
  //Serial.println(BLEDevice::getAddress().toString().c_str());



}

void loop() {
display.clearDisplay(); 
  switch (currentState) {
    
    case  Start_State:
      // ACTION: Ensure LED is OFF
      digitalWrite(LED_PIN, LOW);
      
        currentState = Trans_State;

        printLine(1, "Start>:", true);
        printLine(4, "Trans", false);
        display.display(); // puts text on screen
        delay(2000);// 5 seconds
        clk = 0;

      
      break;
      
    case Trans_State:
      // ACTION: Ensure LED is ON
      digitalWrite(LED_PIN, HIGH);
      
     
        //currentState =  Delay_State; // change to if statement for both delays

        printLine(1, "Trans>:", true);
        printLine(3, "1. Probnation", false);
        printLine(4, "2. Taps", false);
        display.display();
        if(digitalRead(Tap_Pin)== LOW){
          currentState =  Delay_State;
        }
        if(digitalRead(Rot_Pin)== LOW){
          currentState =  Delay_State2;
          
        }

        //delay(2000);
          //display.clearDisplay(); 
      
      break;
      
    // You can add a third state for different timing/actions
    case Taps_State:
    SensorData = analogRead(Sensor_pin); // reads adc value
    //float voltage = sensorData *(3.3/4095.0);
    if(count<10){
     printLine(1, "Tap>End ", true);
     printLine(2, "Count: "+ String (count), true);
     display.display();
     if(SensorData> 80 && !pressed) //if adc value greater than 120 and hasn't been pressed
     {
      count++;
      pressed = true;
       
     }
     currentState =  Taps_State;
    }
    else
    {
      currentState =  End_State;
      count = 0;
    }
    delay(150);

    if(SensorData<80 && pressed)
    {
      pressed = false;
    }
      
      break;
    
    case Rot_State:
      printLine(1, "Rot>End ", true);
      printLine(3,BLEDevice::getAddress().toString().c_str(),true);
      display.display();
      if(count2 > 11){
      currentState =  End_State;
      }
      else
      {
              currentState = Rot_State;
      }
      if(digitalRead(Rot_Pin)== LOW)
      {
        count2++;
        printLine(2, "Count: "+ String (count2), true);
       display.display();
      }
      if (deviceConnected) {
        String str = String(count2);
        pCharacteristic->setValue(str.c_str());
        
        // Give the stack a tiny breath (optional but helps some phones)
        delay(10); 
        
        pCharacteristic->notify(); 
        Serial.println("Notification sent!");
      }

      // Wait for button release so it doesn't spam
      while(digitalRead(Rot_Pin) == LOW);
      break;

    case End_State:
    digitalWrite(LED_PIN, HIGH);
       currentState =  Start_State;
       count2=0;
       printLine(1, "End>", true);
       printLine(2, "Start", true);
     display.display();
     delay(2000);
      break;
    case Delay_State:
    digitalWrite(LED_PIN, LOW);
    if(clk<4){
     printLine(1, "Delay:"+ String (clk)+ "sec", true);
     display.display();
     currentState =  Delay_State;
     clk++;
     delay(1000);

    }
    else
    currentState =  Taps_State;
      
      break;
    case Delay_State2:
    if(clk<4){
     printLine(1, "Delay:"+ String (clk)+ "sec", true);
     display.display();
     currentState =  Delay_State2;
     clk++;
     delay(1000);

    }
    else
    currentState =  Rot_State;
      
      break;

    // Optional: Default case handles unexpected state values
    default:
      currentState =  Start_State; 
      break;
  }
  
  // Any code outside the switch/case runs continuously regardless of state
}

// with textsize = 2  we have 4 lines of text we can use so from 1-4 lines for the oled
// this function compacts the loop while being able to easily write text
//if true text = white if false inverted
void printLine(int lineNumber, const String& text, bool white) {
     //Line 1 
    int y_pos = (lineNumber - 1) * 16;


    if (lineNumber < 1 || lineNumber > 4) {
        return; 
    }

    // 2. Set the cursor to the start of the specified line (X=0, Y=y_pos).
    display.setCursor(0, y_pos);

    // 4. Set color and print the new text.
    if(white)
    {
      display.setTextColor(SSD1306_WHITE);
    }
    if(!white)
    {
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    }
    
    display.print(text);
}
