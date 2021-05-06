
#include <FastLED.h>  //FastLED 
#include <Adafruit_GFX.h> //For 7 seg display
#include "Adafruit_LEDBackpackAlt.h"
//#include <Wire.h>

/*
 * General IO pin definitions
 */
#define SDINERTIA 2   //Shutdown Circuit Inertia Switch
#define SDBOTS 3   //shutdown Circuit Brake over travel switch
#define SDDASH 4   //Shutdown Circuit Dash mounted E-Stop
#define APPSTRAIL 5   //APPS Trail braking error
#define APPSBOUND 6   //Apps boundary condition error
#define APPSDIS 7   //APPS throttle output status
#define BRAKESW 8   //Brake pedal switch
#define RTDGATE 9   //RTD throttle gate output - active high will enable throttle signal to be sent to rear of car
#define RTDBUTTON 10  //RTD dash button input - enables throttle when pressed while brakes are on
#define SPARE2 11  //
#define TEENSY_LED   13   //Onboard Teensy 4 LED, as per hardware
#define ANATHROTTLE 14   //Analog throttle input
#define ANASPARE 15   //
#define LED_DATA_PIN 20  //FastLED output pin

/**** 
 * FastLED Setup - requires #include <FastLED.h>
 * Defines a number of LEDS, then creates an array of CRGB objects for the LEDS
 */
 #define NUM_LEDS 10
 CRGB leds[NUM_LEDS];
 bool flChanged = true;
 //End of FastLED Setup

 /*****
  * Adafruit 7seg display setup
  * Defies a few things for testing, and declares the 7seg object. 
  * Required to run: as we are using i2c port 1, please use the custom 
  * Adafruit_LEDBackapckAlt library. This replaces calls to Wire with calls to Wire1
  */
  int dashTestCount;
  long timeElapsed=0;
  Adafruit_7segment dashLeft = Adafruit_7segment();
  //End 7 seg display setup
  
void setup() {
  delay(2000);
  //FastLED Initialisation
  FastLED.addLeds<WS2811, LED_DATA_PIN, RGB>(leds, NUM_LEDS);
  //End FastLED Initialisation

  //Adafruit 7seg initialisation
  dashTestCount = 0;
  dashLeft.begin(0x70);
  //End Adafruit 7seg initialisation

  /*IO Initialisation
   * 
   */
   pinMode(SDINERTIA,INPUT);
   pinMode(SDBOTS,INPUT);
   pinMode(SDDASH,INPUT);
   pinMode(APPSTRAIL,INPUT);
   pinMode(APPSBOUND,INPUT);
   pinMode(APPSDIS,INPUT);
   pinMode(RTDGATE,INPUT);
   pinMode(BRAKESW,INPUT);
}

void loop() {
  //FastLED display code
  if (flChanged)
  {
    FastLED.show();
    flChanged = false;
  }
  //End FastLED display code

  //Dash display code
  EVERY_N_MILLISECONDS(50){  //Using the FlexCAN non blocking delay
    //dashLeft.print((int)compressInputs());
    timeElapsed = compressInputs();
    dashLeft.print(timeElapsed);
    //timeElapsed++;
    dashLeft.writeDisplay();
  }
}

/*
 * uint8_t compressInputs()
 * Reads the digital IO ports defined at start of program, stores them bitwise in a single byte unsigned int
 */

int compressInputs()
{
  int inputs = 0;
  if (digitalRead(SDINERTIA) == HIGH)
    inputs += 1;
  if (digitalRead(SDBOTS) == HIGH)
    inputs += 2;
  if (digitalRead(SDDASH) == HIGH)
    inputs += 4;
  if (digitalRead(APPSTRAIL) == HIGH)
    inputs += 8;
  if (digitalRead(APPSBOUND) == HIGH)
    inputs += 16;
  if (digitalRead(APPSDIS) == HIGH)
    inputs += 32;
  if (digitalRead(BRAKESW) == HIGH)
    inputs += 64;
  if (digitalRead(RTDGATE) == HIGH)
    inputs += 128;
  return inputs;
}






 
