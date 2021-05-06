//Dashboard CANbus node
//Adapted from Nic Rodgers 2019 BMS translator. See wiki
//Translates BMS can messages from 500k to 250k baud rate
//processes useful data out of BMS CAN messages
//Native 2 Protocol in bms software

#include <circular_buffer.h>
#include <FlexC7AN_T4.h>
#include <imxrt_flexcan.h>
#include <kinetis_flexcan.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t BMS_inMsg,CONT_outMsg1,CONT_outMsg2;
int byte1,byte2,byte3,byte4,byte5,byte6,byte7,byte8,byte9,byte10,byte11,byte12,byte13,byte14,byte15,byte16;
bool flag=0,write_flag=0;

//define interrupt
IntervalTimer print_Timer;

//define functions
void print_Timer_toggle(void);
void initCanMsg(CAN_message_t *msg, uint8_t length, int addr);

void setup() {
  Serial.begin(9600);

  //defines the timer to trigger every 20ms, by running the "print_Timer_toggle" function 
  print_Timer.begin(print_Timer_toggle,2000);
  
  can1.begin();
  can1.setBaudRate(250000);
  can2.begin();
  can2.setBaudRate(500000);

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
  //process BMS CAN messages and rebroadcast onto main bus
  if(can2.read(BMS_inMsg)) {
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
      can1.write(CONT_outMsg1);          //Write Message 1 to Teensy CAN Bus
    }
    else
    {
      can1.write(CONT_outMsg2);          //Write Message 2 to Teensy CAN Bus​
    }
    write_flag=0;
  }

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
