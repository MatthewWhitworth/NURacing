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
 *            (1051 T/E) and logic level shifters
 *   Hardware: - Teesny 3.6
 *             - CCU Breakout Board (See team documents for KiCad files)
 *   Author: Nic Rodgers - c3206083
 *   Date:  2-10-19
 *  ******************************************
 */

 /***********************************
 *  Define all necessary libraries  *
 ***********************************/
#include <FlexCAN_T4.h>  //CAN -> C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\FlexCAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

/***********************************
 *  Define all necessary pins      *
 ***********************************/
#define TEENSY_LED   13   //Set pin 13 as the onboard Teensy 3.6 LED, as per hardware

/***********************************
 *  Define all variables           *
 ***********************************/
#define CAN_BAUD 250000
static CAN_message_t msg;
CAN_message_t inMsg;

void setup() {
 delay(100);
 
  //Open serial communications and wait for port to open:
  Serial.begin(9600);  

  //Turning LED on whenever the Teensy has power
  pinMode(TEENSY_LED,OUTPUT);
  digitalWrite(TEENSY_LED,HIGH);

  //Initialising CAN
  Can0.begin(CAN_BAUD); //initialise CAN0 -> Param Can0 is defined in <FLEXCAN.h>
  Can1.begin(CAN_BAUD); //initialise CAN1 -> Param Can1 is defined in <FLEXCAN.h>

  msg.ext = 0;
  msg.id = 0x100;
  msg.len = 8;
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

void loop(void) 
{
  Can1.write(msg);
  if(Can0.available()) 
  {
    Can0.read(inMsg);
    Serial.print("CAN bus 0: ");
    for(int i=0;i<8;i++)
    {
      Serial.print(inMsg.buf[i]);
      Serial.print(", ");
    }
    Serial.println();
  }
  msg.buf[0]++;
  delay(20);
}
