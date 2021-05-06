/*
	rtd.h 
	Globals and helper functions for the RTD / throttle gate system 
	Assumes presence of a uint8_t shutdownState which represents the state of the SDC
*/
#ifndef RTD
#define RTD
#include "defines.h"
//#include "rtd"
bool isRTD = 0;
bool RTDCheckFlag = 0;	//Flag to run the RTD debounce sequence
bool RTDOverride = 0;	//If set, the RTD will never be disengaged
uint8_t RTDOverrideCount = 0;
int RTDBounces = 0;		//Debounce test count for RTD disable

//Helper function prototypes
bool checkRTD();
void enableRTD(bool*);
void disableRTD(bool*);


#endif