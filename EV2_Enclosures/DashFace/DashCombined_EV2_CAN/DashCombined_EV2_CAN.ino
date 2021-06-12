/*
 * First pass combined software for the dash display driver. 
 * CAN - 
 * 7-seg - 
 * FastLED - waiting on LED strip
 * IO - reading via compressInputs()
 * RTD - Output disabled (protection while probing IO panel)
 * 
 * This sketch will demonstrate the full function of the Dash controller by sending input data from the IO panel
 * from Can2 to Can1 and displaying the result on the 7 segment display.
 * When LEDS arrive, will also reflect the status of the IO panel on addressable LEDs
 */

 /*Includes go here*/
#include "defines.h" //File of #define statements
#include "interface.h"	//Human interface section - FastLED and 7 seg 
#include "rtd.h"
#include "canHelper.h"




/*
  General global variables as required
  throttleValue - accepts a value from the ADC to represent throttle position
*/
uint8_t adcStore[4];
uint8_t throttleValue;
uint8_t runMode = 0;
//uint8_t to capture the state of the SDC
uint8_t shutdownState = 255;	//Init to =0 for production
uint8_t SC_Rear = 0;

//Dash bit HFs from LSB to MSB: BSPD BMS IMD DIS HFL NULL NULL NULL
bool HF_States[] = {0, 0, 0, 0, 0, 0, 0, 0};
//BSPD BMS IMD DIS
int HF_Led_States[] = {1, 1, 1, 1};
int batV = 0;
int highTemp = 0;
int rMTemp, lMTemp;
//End of globals

//LED testy
uint8_t ledIndex = 0;

//Prototypes
void updateShutdown(uint8_t *sdlocation);

CAN_message_t bus1_in,BMS_inMsg,CONT_outMsg1,CONT_outMsg2;
int byte1,byte2,byte3,byte4,byte5,byte6,byte7,byte8,byte9,byte10,byte11,byte12,byte13,byte14,byte15,byte16;
bool flag=0,write_flag=0;

//Create a canHelper object for MB1
canHelper helper1;

//define interrupt
IntervalTimer print_Timer;

//define functions
void print_Timer_toggle(void);
void initCanMsg(CAN_message_t *msg, uint8_t length, int addr);

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
  pinMode(BRAKEPRESFRONT,INPUT);
	pinMode(APPSDIS,INPUT);
	pinMode(BRAKESW,INPUT);
	pinMode(RTDGATE,OUTPUT);    //Pin left as input for test mode. Output for production
	digitalWrite(RTDGATE,LOW);

	pinMode(RTDBUTTON,INPUT);
	pinMode(SPARE2,INPUT);
	//pinMode(TEENSY_LED,OUTPUT);
	pinMode(ANATHROTTLE,INPUT);
	pinMode(LED_DATA_PIN,OUTPUT);
	digitalWrite(LED_DATA_PIN,LOW);
	delay(100);   //short delay to allow peripherals to initialise

	//Adafruit 7seg initialisation
	Wire1.setClock(100000);
	dashLeft.begin(0x70);
	//End Adafruit 7seg initialisation

  Serial.begin(9600);

  //defines the timer to trigger every 20ms, by running the "print_Timer_toggle" function 
  print_Timer.begin(print_Timer_toggle,2000);

	//Initialise CAN controllers
	Can1.begin();
	Can1.setBaudRate(250000);
	Can2.begin();
	Can2.setBaudRate(500000);

	//Initialise the transmit CAN message.
	//Message data: Throttle, RTDState, ShutdownState, 5x Unused
	initCanMsg(&toRearMsg,8,0x0000100);

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

  //Defining Message 1 output for the Teensy 
  // Contains the following data:
  //{Min Cell Voltage byte 1,Min Cell Voltage byte 2,Max Cell Voltage byte 1,Max Cell Voltage byte 2,
  // Min Cell Voltage Index,Min Cell Temperature,Max Cell Temp, Max Cell Temp Index}
  initCanMessage(&CONT_outMsg1, 8, 0x0111110);
  CONT_outMsg1.buf[0] = byte1;
  CONT_outMsg1.buf[1] = byte2;
  CONT_outMsg1.buf[2] = byte3;
  CONT_outMsg1.buf[3] = byte4;
  CONT_outMsg1.buf[4] = byte5;
  CONT_outMsg1.buf[5] = byte6;
  CONT_outMsg1.buf[6] = byte7;
  CONT_outMsg1.buf[7] = byte8;

  // Defining Message 1 output for the Teensy 
  // Contains the following data:​
  // {Shunt Voltage byte 1,Shunt Voltage byte 2,Shunt Current byte 1,Shunt Current byte 2,
  //  Shunt Current byte 3,Shunt Current byte 4,Critical Failure Flags 1, Critical Failure Flags 2}
  initCanMessage(&CONT_outMsg2, 8, 0x0111120);
  CONT_outMsg2.buf[0] = byte9;
  CONT_outMsg2.buf[1] = byte10;
  CONT_outMsg2.buf[2] = byte11;
  CONT_outMsg2.buf[3] = byte12;
  CONT_outMsg2.buf[4] = byte13;
  CONT_outMsg2.buf[5] = byte14;
  CONT_outMsg2.buf[6] = byte15;
  CONT_outMsg2.buf[7] = byte16;

}



void loop() {
	//Do in any mode 
	EVERY_N_MILLISECONDS(10)
	{
		//Update data temp variables
		adcStore[0] = analogRead(ANATHROTTLE);
		adcStore[1] = analogRead(BRAKEPRESFRONT);
		adcStore[2] = 0;
		adcStore[3] = 0;
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
				toRearMsg.buf[4] = digitalRead(APPSTRAIL);
				toRearMsg.buf[5] = digitalRead(BRAKESW);
				toRearMsg.buf[6] = shutdownState;;
				toRearMsg.buf[7] = 0;
				//Send a CAN message to Rear of Car
				Can1.write(toRearMsg);
				
				
				//Dash display update
//        if (rMTemp >= lMTemp){
//          highTemp = rMTemp;
//        }
//        else {
//          highTemp = lMTemp;
//        }
				dashLeft.print(highTemp);
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
				updateTrailBrakeL(digitalRead(APPSTRAIL));
				
				//LED display update
				FastLED.show();
				//Can2.write(toRearMsg);
				
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
  
  //process BMS CAN messages and rebroadcast onto main bus
  BMSProcess();

  //Read and process messages from bus 1
  Bus1Read();

  //Determine Hard Fault states from can message
  byteUnpack();

  //Update dash HF led's
  updateHardFaults();
  
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
	if (HF_States[0] == HIGH)
		tempSDS |= 0x08;
	if (HF_States[1] == HIGH)
		tempSDS |= 0x10;
	if (HF_States[2] == HIGH)
		tempSDS |= 0x20;
	if (HF_States[3] == HIGH)
		tempSDS |= 0x40;
	if (HF_States[4] == HIGH)
		tempSDS |= 0x80;
	*sdLocation = tempSDS;
}

/***************************************************************************************************
* This is the timer function that is run every 20ms. Each iteration it toggles the boolean "flag"  *
* which indicates which message is written per code iteration. The "Write flag" boolean is also *
* set to high every time it's called. This flag dictates when the timer has elapsed, and it is time *
* for the code to poll the next message onto the Teensy's CAN Bus                                            *
***************************************************************************************************/
void print_Timer_toggle(void)
{
  flag=!flag;
  write_flag=1;
}

//CAN message initialiser
void initCanMessage(CAN_message_t *msg, uint8_t length, int addr)
{
  msg->id = addr;
  msg->len = length;
  msg->flags.extended = 1;
  msg->flags.remote   = 0;
  msg->flags.overrun  = 0;
  msg->flags.reserved = 0;
  for (int i=0;i<length;i++)
  {
    msg->buf[i] = 0;  //Init the message to zeros
  }
}

void BMSProcess(void) {
  if(Can2.read(BMS_inMsg)) {
      if(BMS_inMsg.id==0x00111100)
      {
        byte1=BMS_inMsg.buf[0];    //Minimum Cell Voltage LSB
        byte2=BMS_inMsg.buf[1];    //Minimum Cell Voltage MSB
        byte3=BMS_inMsg.buf[2];    //Maximum Cell Voltage LSB
        byte4=BMS_inMsg.buf[3];    //Maximum Cell Voltage LSB
        byte5=BMS_inMsg.buf[4];    //Minimum Cell Voltage Index
      }
      else if(BMS_inMsg.id==0x00111200)
      {
        byte6=BMS_inMsg.buf[0];   //Minimum Cell Temperature
        byte7=BMS_inMsg.buf[1];   //Maximum Cell Temperature
        byte8=BMS_inMsg.buf[3];   //Maximum Cell Temperature Index
      }
      else if(BMS_inMsg.id==0x00111500)
      {
        byte11=BMS_inMsg.buf[2];    //Shunt Voltage LSB
        byte12=BMS_inMsg.buf[3];   //Shunt Voltage MSB
        byte9=BMS_inMsg.buf[0];   //SOC%
        byte10=BMS_inMsg.buf[1];   //Shunt Temperature​
        //byte13=BMS_inMsg.buf[6];   //Shunt Current Byte 3 
        //byte14=BMS_inMsg.buf[7];   //Shunt Current Byte 4
      }
      else if(BMS_inMsg.id==0x00140100)
      {
        byte15=BMS_inMsg.buf[2];    // Critical Flags LSB
        byte16=BMS_inMsg.buf[3];    // Critical Flags MSB
      }
   }

  // Build up Teensy CAN message 1
  CONT_outMsg1.buf[0] = byte1;
  CONT_outMsg1.buf[1] = byte2;
  CONT_outMsg1.buf[2] = byte3;
  CONT_outMsg1.buf[3] = byte4;
  CONT_outMsg1.buf[4] = byte5;
  CONT_outMsg1.buf[5] = byte6;
  CONT_outMsg1.buf[6] = byte7;
  CONT_outMsg1.buf[7] = byte8;    
  // Build up Teensy CAN message 2
  CONT_outMsg2.buf[0] = byte9;
  CONT_outMsg2.buf[1] = byte10;
  CONT_outMsg2.buf[2] = byte11;
  CONT_outMsg2.buf[3] = byte12;
  CONT_outMsg2.buf[4] = byte13;
  CONT_outMsg2.buf[5] = byte14;
  CONT_outMsg2.buf[6] = byte15;
  CONT_outMsg2.buf[7] = byte16;

  //Iterate between writing message 1 and message 2 to the Teensy CAN Bus every 20ms once the timer has elapsed
  if(write_flag)
  {
    if(flag)
    {
      Can1.write(CONT_outMsg1);          //Write Message 1 to Teensy CAN Bus
    }
    else
    {
      Can1.write(CONT_outMsg2);          //Write Message 2 to Teensy CAN Bus​
    }
    write_flag=0;
  }
}

void Bus1Read() {
  if(Can1.read(bus1_in)) {
    if(bus1_in.id==0x00001100){
      SC_Rear = bus1_in.buf[7];
    }
    else if (bus1_in.id==0x00001001){
      batV = bus1_in.buf[0]*256 + bus1_in.buf[1];
    }
//    else if (bus1_in.id==0xCF10F05) { //right controller
//      rMTemp = bus1_in.buf[2] - 40;
//    }
//    else if (bus1_in.id==0xCF11F05) { //left controller
//      lMTemp = bus1_in.buf[2] - 40;
//    }
  }
}

//Fucntion unpacks bits from CAN message bytes
//Sequentially removes MSB from byte and assigns flags to given array as output
//TODO: Make universal in that it can receive and unpack any array and input byte
// CHECK BIT ORDERING
void byteUnpack() {
  int i;
  uint8_t byteTemp = SC_Rear;     //tracks reduction of 1 bits
  uint8_t MSB = 128;              //Starting breakpoint for MSB
  for (i=7;i>=0;i--){
      if (byteTemp >= MSB){       //bit = 1
        HF_States[i] = 1;
        byteTemp -= MSB;
      }
      else {                      //bit = 0
        HF_States[i] = 0;
      }
      MSB = MSB/2;                //Reduce to next MSB breakpoint
  }
}

void updateHardFaults(){
  int i;
  
  for (i=0; i<=3; i++){
    switch (HF_Led_States[i]) {
    case 1:
      resetHardFaultL(i);
      if (HF_States[i] == 0){
        HF_Led_States[i] = 3;
      }
      break;
    case 2:
      standbyHardFaultL(i);
      if (HF_States[i] == 0){
        HF_Led_States[i] = 3;
      }
      else if (HF_States[4] == 1){
        HF_Led_States[i] = 1;
      }
      break;
    case 3:
      setHardFaultL(i);
      if (HF_States[i] == 1){
        HF_Led_States[i] = 2;
      }
      break;
    }
  }
}
