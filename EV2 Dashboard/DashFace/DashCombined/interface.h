/*
 * interface.h 
 * Methods and data used by the human interface - dash 7 seg display and LEDs
 */

#include <FastLED.h>  //FastLED 
#include <Adafruit_GFX.h> //For 7 seg display
#include "Adafruit_LEDBackpackAlt.h"  //Customised Adafruit library that addresses i2c 1


/*
 * Adafruit 7seg initialisation
 */
Adafruit_7segment dashLeft = Adafruit_7segment();

/*
 * FastLED Setup - requires #include <FastLED.h>
 * Defines a number of LEDS, then creates an array of CRGB objects for the LEDS
 */
 CRGB leds[NUM_LEDS];
 //End of FastLED Setup
 
 uint8_t gHue = 0;	//Used for Demo mode and initialisation 
 
 /*
 LED update prototypes
 Call from main loop to update LEDs as required
 */
 
 void lostCANL();
 //Display a lost can comms pattern - blue battery, others TBD
 
 void updateBatL(uint8_t batState);
 //Update the three battery charge state indicators
 //Params batState percentage of charge remaining
 
 void setHardFaultL(uint8_t LEDno);
 //Set one of the Hard Fault LEDs on
 //Params LEDNo index from 0 to 4 of LED to illuminate
 
 void clearHardFaultL();
 //Revert the array of hard fault indicators to green
 
 void updateSDL(uint8_t sdState);
 //Update the shutdown indicator LED
 //Params sdState the status word of the Shutdown circuit
 //Sets light to red if off, orange if precharge, green if active
 //Maybe mirror TSAL behaviour instead?
 
 void updateMotorTempL(uint8_t tempIn);
 //Update the tempreature LED based on tempIN vs motor_warn, motor_danger
 
 void updateRTDL(bool active);
 //Update the RTD LED to reflect the state of the Throttle Gate
 
 void updateTrailBrakeL(bool active);
 //Update the trail brake light if we are brake gated