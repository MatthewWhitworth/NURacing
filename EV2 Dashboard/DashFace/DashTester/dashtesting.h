//Include guard
#ifndef DASHTESTING
#define DASHTESTING

//Test or production mode definition
//#define TESTMODE

//Second CAN interface, used to send and receive test data
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
//static uint8_t hex[17] = "0123456789abcdef";

#include "dashtesting.cpp"
#endif