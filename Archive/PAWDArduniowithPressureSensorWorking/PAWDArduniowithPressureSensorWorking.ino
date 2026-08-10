#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// --- Configuration ---
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64   // OLED display height, in pixels
#define OLED_RESET -1      // Reset pin (-1 if sharing Arduino Reset pin)
//#define SCREEN_ADDRESS 0x3C // I2C address (Try 0x3D if 0x3C fails) //for 0.96 in oled
#define SCREEN_ADDRESS 0x3D //1.3inch oled from school


// Initialize the display object
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

// Pin for the LED
const int LED_PIN = 32;// can be removed just inital bug testing
const int Sensor_pin = 34; //Pressure sensor pin
const int Rot_Pin = 12; // external buttons
const int Tap_Pin= 14; //external buttons
//int clock = 0;


int clk = 0; //for delay
int count = 0; // amount of taps
int SensorData=0; //initalizing pressure sensor to zero 
bool pressed = false; //prevents overcounting in taps
void setup() {
   delay(1000); //one second delay to allow oled to run/intialize without pc
  pinMode(LED_PIN, OUTPUT); //can be removed just sending a signal to led to see if it works

  pinMode(Sensor_pin,INPUT); //pressure sensor pin
  pinMode(Rot_Pin,INPUT); //setting pins to take input
  pinMode(Tap_Pin,INPUT);
  // Set the initial state when the program starts
  currentState =  Start_State; 
  Serial.println("FSM Starting in  Start_State");


  // Start I2C communication
  Wire.begin(); 

  // Initialize the SSD1306 display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // If initialization fails, print error and stop
    Serial.begin(115200);//baud rate
    Serial.println(F("SSD1306 initialization failed"));
    for(;;); 
    
  }

  
  // 1. Clear the entire display buffer
  display.clearDisplay(); 
  display.setTextSize(2);         // Set text size to 2 (larger text)
  display.setTextColor(SSD1306_WHITE); // Set text color to white
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
        if(digitalRead(Rot_Pin)== HIGH){
          currentState =  Delay_State;
        }
        if(digitalRead(Tap_Pin)== HIGH){
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
      display.display();
      delay(1000);
      currentState =  End_State;
      break;

    case End_State:
    digitalWrite(LED_PIN, HIGH);
       currentState =  Start_State;
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
