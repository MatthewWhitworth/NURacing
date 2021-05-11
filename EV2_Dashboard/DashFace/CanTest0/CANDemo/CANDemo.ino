#include <FlexCAN_T4.h>

/* 
 *  ******************************************
 *  Project:  CAN Test Code
 *  Function: Ensure that CAN buses are able to
 *            communicate between each other and
 *            prove hardware works as it should
 *            Hardware should be set so CANH from 
 *            CAN1 is connected to CANH from CAN0 
 *            and same for CANL. These should 
 *            travel through a CAN transciever 
 *            (1051 T/3)
 *   Hardware: - Teesny 4
 *             - FCU Dash Driver shield (Dashboard Face V0)
 *   Author: Ddavid Birdsall - 3027249
 *   Date:  20.1.2020
 *   Thanks to Nic R for pioneering this stuff
 *  ******************************************
 */

 /***********************************
 *  Define all necessary libraries  *
 ***********************************/


/***********************************
 *  Define all necessary pins      *
 ***********************************/
#define TEENSY_LED   13   //Set pin 13 as the onboard Teensy 3.6 LED, as per hardware

/***********************************
 *  Define all variables           *
 ***********************************/
#define CAN_BAUD 500000
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
static CAN_message_t msg;
static uint8_t hex[17] = "0123456789abcdef";

void setup() {
 delay(100);
 
  //Open serial communications and wait for port to open:
  Serial.begin(9600);  

  //Turning LED on whenever the Teensy has power
  pinMode(TEENSY_LED,OUTPUT);
  digitalWrite(TEENSY_LED,HIGH);

  //Initialising CAN
  Can2.begin();
  Can1.begin();
  Can1.setBaudRate(CAN_BAUD);
  Can2.setBaudRate(CAN_BAUD);

  
  //msg.ext = 0; - commented for T4
  msg.id = 0x100;
  msg.len = 8;
  msg.flags.extended = 0;
  msg.flags.remote   = 0;
  msg.flags.overrun  = 0;
  msg.flags.reserved = 0;
  msg.buf[0] = 10;
  msg.buf[1] = 20;
  msg.buf[2] = 0;
  msg.buf[3] = 100;
  msg.buf[4] = 128;
  msg.buf[5] = 64;
  msg.buf[6] = 32;
  msg.buf[7] = 16;
  Serial.print("Initialise Complete");
}

// -------------------------------------------------------------
void loop(void)
{
  Serial.println("T4.0cantest - Repeat: Read bus2, Write bus1");
  CAN_message_t inMsg;
  while (Can2.read(inMsg)!=0) 
  {
    Serial.print("W RD bus 2: "); hexDump(8, inMsg.buf);
  }
  msg.buf[0]++;
  Can1.write(msg);
  msg.buf[0]++;
  Can1.write(msg);
  msg.buf[0]++;
  Can1.write(msg);
  msg.buf[0]++;
  Can1.write(msg);
  msg.buf[0]++;
  Can1.write(msg);  
  delay(20);
}

//Borrowed from the 'net
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working>>4 ] );
    Serial.write( hex[ working&15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
}
