/*
 * First pass combined software for the dash display driver. 
 * CAN - 
 * 7-seg - 
 * FastLED - waiting on LED strip
 * IO - reading via compressInputs()
 * RTD - Output disabled (protection while probing IO panel)
 * 
 * This sketch will demonstrate the full function of the Dash controller by sending input data from the IO panel
 * from CAN2 to CAN1 and displaying the result on the 7 segment display.
 * When LEDS arrive, will also reflect the status of the IO panel on addressable LEDs
 */

 /*Includes go here*/
#include "defines.h" //File of #define statements
#include "interface.h"	//Human interface section - FastLED and 7 seg 
#include "rtd.h"
#include "canHelper.h"
//If running in test mode, uncomment this
//#include "dashtesting.h"




/*
  General global variables as required
  throttleValue - accepts a value from the ADC to represent throttle position
*/
uint8_t adcStore[4];
uint8_t throttleValue;
uint8_t runMode = 0;
//uint8_t to capture the state of the SDC
uint8_t shutdownState = 255;	//Init to =0 for production
//End of globals

//LED testy
uint8_t ledIndex = 0;

//Prototypes
void updateShutdown(uint8_t *sdlocation);


//Create a canHelper object for MB1
canHelper helper1;


void setup() {
	  /*IO Initialisation
   * 
   * Wire and FastLED will initialise their own pins
   */
	pinMode(SDINERTIA,INPUT);
	pinMode(SDBOTS,INPUT);
	pinMode(SDDASH,INPUT);
	pinMode(APPSTRAIL,INPUT);
	pinMode(APPSBOUND,INPUT);
	pinMode(APPSDIS,INPUT);
	pinMode(BRAKESW,INPUT);
	pinMode(RTDGATE,OUTPUT);    //Pin left as input for test mode. Output for production
	digitalWrite(RTDGATE,LOW);

	pinMode(RTDBUTTON,INPUT);
	pinMode(SPARE2,INPUT);
	pinMode(TEENSY_LED,OUTPUT);
	pinMode(ANATHROTTLE,INPUT);
	pinMode(ANASPARE,INPUT);
	pinMode(LED_DATA_PIN,OUTPUT);
	digitalWrite(LED_DATA_PIN,LOW);
	delay(100);   //short delay to allow peripherals to initialise

	//Adafruit 7seg initialisation
	Wire1.setClock(100000);
	dashLeft.begin(0x70);
	//End Adafruit 7seg initialisation



	//Initialise CAN controllers
	Can1.begin();
	Can1.setBaudRate(CAN_BAUD);
	Can2.begin();
	Can2.setBaudRate(CAN_BAUD);

	//Initialise the transmit CAN message.
	//Message data: Throttle, RTDState, ShutdownState, 5x Unused
	initCanMsg(&toRearMsg,8,67);
/*
	//Set up a can mailbox to receive (presently test) data
	Can2.enableMBInterrupt(MB1);
	Can2.setMBFilter(MB1,0x100);
	Can2.onReceive(MB1,testCanSniff);
	sei();
	//End of mailbox configuration
	*/

	//Configure ADC
	analogReadRes(8);
	analogReadAveraging(1);
	//End ADC configuration
	 
	digitalWrite(TEENSY_LED,HIGH);
	
	//FastLED Initialisation
	FastLED.addLeds<1, WS2812, LED_DATA_PIN, RGB>(leds, NUM_LEDS);
	//End FastLED Initialisation
	//Set default states for addressable LEDs
	fill_solid(leds,NUM_LEDS, 0x0f0f0f);
	FastLED.setBrightness(LED_BRIGHTNESS);
	
	//Run mode initialisation 
	if (digitalRead(RTDBUTTON) == HIGH)
		runMode = 1;
}



void loop() {
	//Do in any mode 
	EVERY_N_MILLISECONDS(10)
	{
		//Update data temp variables
		adcStore[0] = analogRead(ANATHROTTLE);
		adcStore[1] = analogRead(ANASPARE);
		/*adcStore[2] = analogRead(ANABRK1);
		adcStore[3] = analogRead(ANABRK2);*/
		updateShutdown(&shutdownState);
	}
	
	
	//Master status - select between demo mode, startup, and other modes as required
	switch ((int)runMode){
		case 0:		//Demo mode - sweep LEDs, lock throttle off
			//Update every 10 millis 
			EVERY_N_MILLISECONDS(10)
			{
				//Refresh the dash with a simple EV2 logo
				dashLeft.writeDigitRaw(0,0b1111001);
				dashLeft.writeDigitRaw(1,0b11100);
				dashLeft.writeDigitNum(4,2);
				dashLeft.writeDisplay();
				//Update the LEDs
				gHue++;
				fill_rainbow(leds,NUM_LEDS,gHue,14);
				FastLED.show();
			}
		break;
		case 1:		//Startup mode - Show welcome to EV2, fancy LED pattern etc.
			runMode = 2;
		break;
		case 2:		//Run mode - throttle can be armed, car operational
		
			EVERY_N_MILLISECONDS(50)
			{
				//Collect data ready for a CAN message transmission
				toRearMsg.buf[0] = adcStore[0];
				toRearMsg.buf[1] = adcStore[1];
				toRearMsg.buf[2] = adcStore[2];
				toRearMsg.buf[3] = adcStore[3];
				//toRearMsg.buf[4] = shutdownState;
				toRearMsg.buf[4] = 16;
				toRearMsg.buf[5] = 32;
				toRearMsg.buf[6] = 64;
				toRearMsg.buf[7] = 128;
				//Send a CAN message to Rear of Car
				Can1.write(toRearMsg);
				
				
				//Dash display update
				dashLeft.print((int)shutdownState);
				cli();
				dashLeft.writeDisplay();
				sei();
				
				//If not RTD, see if we should be 
				if(255 == shutdownState && isRTD == false)	//If the SDC is fully enabled AND we are not Ready to Drive
				{
					if(checkRTD() == true)	//If all conditions 
						enableRTD(&isRTD);
						
				}else if (isRTD && RTDOverride == 0)	//If we are Ready to Drive and not overriden, see if we need to not be 
				{
					if (shutdownState != 255)
						//disableRTD(&isRTD);
						RTDCheckFlag = 1;		//Disable moved below, this enables simple debounce check
				}
				
				//See if CAN connection has timed out
				if (helper1.heartbeat())
				{
					updateBatL(250);
				}
				else
				{
					fill_solid(leds,3,CRGB(0,0,100));
				}
				//Update data shown on addressable LEDs
				updateSDL(shutdownState);
				updateRTDL(isRTD);
				clearHardFaultL();
				updateTrailBrakeL(digitalRead(APPSTRAIL));
				
				//LED display update
				FastLED.show();
				Can2.write(toRearMsg);
				
				//Check for RTD Override activation on a low frequency
				EVERY_N_MILLISECONDS(500){
					
					if (digitalRead(RTDBUTTON) == LOW)
					{
						RTDOverrideCount++;
						if (RTDOverrideCount > 10)
							RTDOverride = 1;
					}
					else {
						RTDOverrideCount = 0;
					}
				}
			}
		break;
	}
	
	
	EVERY_N_MILLISECONDS(6)
	{
		//Update the state of the Shutdown circuit
		//TODO
		
		
		//Update the dash display - for now, output diagnostic data 
		//IOBits = collectInputs();
		//digitalWrite(TEENSY_LED,LOW);
		
		Can2.events();	//Call the CAN event handler to enable CAN interrupt callback functions
	}
	
	//Shutdown circuit debounce
	if (isRTD && RTDCheckFlag)
	{
		//Check if the SDC has bounced back to 255
		if (shutdownState == 255)
		{
			RTDBounces = 0;
			RTDCheckFlag = 0;
		}else
		{
			RTDBounces++;	//Increment the bounce counter - at a defined value, disaboe the throttle
			if (RTDBounces > RTD_TOTALBOUNCE)
			{
				//Disable the throttle, and reset the flags
				RTDBounces = 0;
				RTDCheckFlag = 0;
				disableRTD(&isRTD);
			}
		}
	}
  
}

/*User defined functions:
 * int compressInputs()
 */



//Interrupt callback for test CAN message.  Only call if TESTMODE is enabled
void testCanSniff(const CAN_message_t &inMsg)
{
  //hexDump(8, inMsg.buf);
  IOBits = inMsg.buf[0];
}

//RTD helper functions in other file 

/*Update the state of the Shudtdown Circuit int
void, param pointer to shutdown variables
*/
void updateShutdown(uint8_t *sdLocation)
{
	uint8_t tempSDS = 0;
	//Inertia Switch
	if (digitalRead(SDINERTIA) == HIGH)
		tempSDS |= 0x01;
	//Brake Overtravel Switch
	if (digitalRead(SDBOTS) == HIGH)
		tempSDS |= 0x02;
	//Dash E-Stop Switch
	if (digitalRead(SDDASH) == HIGH)
		tempSDS |= 0x04;
	//Other parts of the SDC to be retrieved via CAN later
	if (true)
		tempSDS |= 0x08;
	if (true)
		tempSDS |= 0x10;
	if (true)
		tempSDS |= 0x20;
	if (true)
		tempSDS |= 0x40;
	if (true)
		tempSDS |= 0x80;
	*sdLocation = tempSDS;
}

 
