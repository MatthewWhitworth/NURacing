//teensy 4.0 canbus test on dash face board
//CAN2.0 mode
//pin 13 led output
//Sending an input analog signal from can1 to can2

#include <circular_buffer.h>
#include <FlexCAN_T4.h>
#include <imxrt_flexcan.h>
#include <kinetis_flexcan.h>
#include "canHelper.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
#define CAN_BAUD 500000
CAN_message_t receiveMsg,sendMsg; //message storage structure

int potPin = 14;
int potPos;
double collectTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(potPin,INPUT);
  
  can1.begin();
  can2.begin();
  can1.setBaudRate(CAN_BAUD);
  can2.setBaudRate(CAN_BAUD);

  //Initialise the transmit CAN message.
  //Message data: Throttle, 7x Unused
  initCanMsg(&sendMsg,8,67);

  //Configure ADC
  analogReadRes(8);
  analogReadAveraging(1);
  //End ADC configuration

}

void loop() {
  if (millis() - collectTime >= 500){
    potPos = analogRead(potPin)*(5.0/3.3);
    Serial.print("Raw input: "); Serial.println(potPos);
    sendMsg.buf[0] = potPos;
    can1.write(sendMsg);
    collectTime = millis();
  }
  if ( can2.read(receiveMsg) ) {
    canSniff(receiveMsg);
  }
}

void canSniff(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}

void readInputs() {
  
}
