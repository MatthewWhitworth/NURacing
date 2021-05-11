/*
  canHelper.h 
  Helper functions and definitions for CANBUS functionality
  */
#include <FlexCAN_T4.h> //FlexCAN for Teensy 4

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
static CAN_message_t sendMsg;
static CAN_message_t toRearMsg;
volatile int IOBits;	//Used to capture IO data into a single var

bool canAlive = false;

bool getCanTimeout();

class canHelper
{
	public:
	//Constructor
	canHelper()
	{
		canAlive = false;
	}
	//No destructor - canHelper not destroyed during system run time
	
	//Return true unless CAN network has dropped
	bool heartbeat()
	{
		return canAlive;
	}
	
	private:
	bool canAlive;
};