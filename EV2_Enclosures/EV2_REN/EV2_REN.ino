//REN CAN controller
//re-identifies right controller message and transmits onto main canbus

#include <circular_buffer.h>
#include <FlexCAN_T4.h>
#include <imxrt_flexcan.h>
#include <kinetis_flexcan.h>
#include "canHelper.h"

#define HF_BMS 5    //BMS status
#define HF_Dis 3    //Discharge status
#define HF_IMD 4    //IMD status
#define HF_BSPD 2   //BSPD status
#define HFL_Out 6   //HFL status

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t msg, msg_sc;

int led = 13;
int beatTime = 0;
int scTime = 0;
uint8_t HF_State;

void setup(void) {
  Serial.begin(9600);
  
  can1.begin();
  can1.setBaudRate(250000);
  can2.begin();
  can2.setBaudRate(250000);

  initCanMsg(&msg_sc, 8, 0x0111100);

  pinMode(HF_BMS,INPUT_PULLDOWN);
  pinMode(HF_Dis,INPUT_PULLDOWN);
  pinMode(HF_IMD,INPUT_PULLDOWN);
  pinMode(HF_BSPD,INPUT_PULLDOWN);
  pinMode(HFL_Out,INPUT_PULLDOWN);
  pinMode(led,OUTPUT);
}

void loop() {
  heartbeat();
  
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

  //every 500ms write SC state to bus
  if (millis() - scTime >= 500){
    HF_State = collectInputs();
    msg.buf[7] = HF_State;
    can1.write(msg_sc);
    scTime = millis();
    Serial.println(HF_State);

    //4inputProcess();
    
  }
  
}

void heartbeat(){
  if (millis() - beatTime >= 500){
    digitalWrite(led, !digitalRead(led));
    beatTime = millis();
  }
}

void inputProcess() {
  //Read data test
    if (HF_State >= 16){
      //HFL ok
      Serial.print("HFL: "); Serial.println("OK");
      HF_State -= 16;
    }
    else {
      Serial.print("HFL: "); Serial.println("Error");
    }
    if (HF_State >= 8){
      //HF_Dis ok
      Serial.print("Discharge: "); Serial.println("OK");
      HF_State -= 8;
    }
    else {
      Serial.print("Discharge: "); Serial.println("Error");
    }
    if (HF_State >= 4){
      //HF_IMD ok
      Serial.print("IMD: "); Serial.println("OK");
      HF_State -= 4;
    }
    else {
      Serial.print("IMD: "); Serial.println("Error");
    }
    if (HF_State >= 2){
      //HF_BMS ok
      Serial.print("BMS: "); Serial.println("OK");
      HF_State -= 2;
    }
    else {
      Serial.print("BMS: "); Serial.println("Error");
    }
    if (HF_State >= 1){
      //BSPD ok
      Serial.print("BSPD: "); Serial.println("OK");
      HF_State -= 1;
    }
    else {
      Serial.print("BSPD: "); Serial.println("Error");
    }
}
