#include <FlexCAN_T4.h>

//read data from motor controllers

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; //left controller
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2; //right controller
CAN_message_t msg1,msg2; //message storage structure

//controller data outputs
int RPM_L,CUR_L,VOL_L,THR_L,MTMP_L,CTMP_L,RPM_R,CUR_R,VOL_R,THR_R,MTMP_R,CTMP_R;
double dataOutputTime = 0;

void setup(void) {
  can1.begin();
  can1.setBaudRate(250000);
  can2.begin();
  can2.setBaudRate(250000);

  dataOutputTime = millis();
}

void loop() {
  

  if ( can1.read(msg1) ) {
    //Left contoller message A
    if(msg1.id==0x0CF11E05)
        {
            //Save Motor Controller Message 1 parameters in here
            RPM_L=(msg1.buf[0]+256*msg1.buf[1]);
            CUR_L=(msg1.buf[2]+256*msg1.buf[3])/10;
            VOL_L=(msg1.buf[4]+256*msg1.buf[5])/10;
        }
    //Left controller message B
    else if(msg1.id==0x0CF11F05)
        {
            //Save Motor Controller Message 2 parameters in here​
            THR_L = (msg1.buf[0]/255.0)*100;  //Avoid integer division by dividing by 255.0
            MTMP_L= (msg1.buf[2]-40);         //Note this says 30 deg. offset in datasheet, see sensor experiment. This is cofirmed to be a 40 deg. offset
            CTMP_L= (msg1.buf[1]-40);         //Note this says 40 deg. offset in datasheet
        }
    else {
      Serial.print("Message ID incorrect");
    }
//    Serial.print("CAN1 "); 
//    Serial.print("MB: "); Serial.print(msg1.mb);
//    Serial.print("  ID: 0x"); Serial.print(msg1.id, HEX );
//    Serial.print("  EXT: "); Serial.print(msg1.flags.extended );
//    Serial.print("  LEN: "); Serial.print(msg1.len);
//    Serial.print(" DATA: ");
//    for ( uint8_t i = 0; i < 8; i++ ) {
//      Serial.print(msg1.buf[i]); Serial.print(" ");
//    }
//    Serial.print("  TS: "); Serial.println(msg1.timestamp);
  }
  else if ( can2.read(msg2) ) {
    //Right contoller message A
    if(msg2.id==0x0CF11E05)
        {
            //Save Motor Controller Message 1 parameters in here
            RPM_R=(msg2.buf[0]+256*msg2.buf[1]);
            CUR_R=(msg2.buf[2]+256*msg2.buf[3])/10;
            VOL_R=(msg2.buf[4]+256*msg2.buf[5])/10;
        }
    //Right controller message B
    else if(msg2.id==0x0CF11F05)
        {
            //Save Motor Controller Message 2 parameters in here​
            THR_R = (msg2.buf[0]/255.0)*100;  //Avoid integer division by dividing by 255.0
            MTMP_R= (msg2.buf[2]-40);         //Note this says 30 deg. offset in datasheet, see sensor experiment. This is cofirmed to be a 40 deg. offset
            CTMP_R= (msg2.buf[1]-40);         //Note this says 40 deg. offset in datasheet
        }
    else {
      Serial.print("Message ID incorrect");
    }
  }

  if((millis() - dataOutputTime) >= 1000) {
    Serial.print("  Left RPM: "); Serial.print(RPM_L);
    Serial.print("  Left Controller Temp: "); Serial.println(CTMP_L);
    Serial.print("  Right RPM: "); Serial.print(RPM_R);
    Serial.print("  Right Controller Temp: "); Serial.println(CTMP_R);
    dataOutputTime = millis();
  }
  
}
