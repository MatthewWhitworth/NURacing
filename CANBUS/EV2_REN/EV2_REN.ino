//REN CAN controller
//re-identifies right controller message and transmits onto main canbus

#include <circular_buffer.h>
#include <FlexCAN_T4.h>
#include <imxrt_flexcan.h>
#include <kinetis_flexcan.h>
#include "canHelper.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t msg;

int led = 13;
int beatTime = 0;

void setup(void) {
  Serial.begin(9600);
  
  can1.begin();
  can1.setBaudRate(250000);
  can2.begin();
  can2.setBaudRate(250000);

  pinMode(13,OUTPUT);
}

void loop() {
  heartbeat();

//  if ( can1.read(msg) ) {
//      Serial.print("CAN1 "); 
//      Serial.print("MB: "); Serial.print(msg.mb);
//      Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
//      Serial.print("  EXT: "); Serial.print(msg.flags.extended );
//      Serial.print("  LEN: "); Serial.print(msg.len);
//      Serial.print(" DATA: ");
//      for ( uint8_t i = 0; i < 8; i++ ) {
//        Serial.print(msg.buf[i]); Serial.print(" ");
//      }
//      Serial.print("  TS: "); Serial.println(msg.timestamp);
//    }
  
  //translate message id if message detected on second bus
  if ( can2.read(msg) ) {
    //right controller message 1
    if (msg.id == 0xCF11E05 ) {
      msg.id = 0xCF10E05;
    }
    //right controller message 2
    else if (msg.id == 0xCF11F05 ) {
      msg.id = 0xCF10F05;
    }
    else {
      Serial.println("Error: Unexpected CAN message received");
    }

    //broadcast message onto main bus with new id
    Serial.print("  ID: 0x"); Serial.println(msg.id, HEX );
    can1.write(msg);
  }
}

void heartbeat(){
  if (millis() - beatTime >= 500){
    digitalWrite(led, !digitalRead(led));
    beatTime = millis();
  }
}
