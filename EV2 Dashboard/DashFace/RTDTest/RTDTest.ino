/*
 * Simple program to demonstrate the functionality of the RTD Daughter card
 * Will automatically leave RTD state 5 seconds after successful enable
 * TEENSY LED used to indicate RTD state
 */

//Define used pins on the Teensy


#define SDINERTIA 2   //Shutdown Circuit Inertia Switch
#define SDBOTS 3   //shutdown Circuit Brake over travel switch
#define SDDASH 4   //Shutdown Circuit Dash mounted E-Stop
#define APPSTRAIL 5   //APPS Trail braking error
#define APPSBOUND 6   //Apps boundary condition error
#define APPSDIS 7   //APPS throttle output status
#define BRAKESW 8   //Brake pedal switch
#define RTDGATE 9   //RTD throttle gate output - active high will enable throttle signal to be sent to rear of car
#define RTDBUTTON 10  //RTD dash button input - enables throttle when pressed while brakes are on
#define SPARE2 11  //
#define TEENSY_LED   13   //Onboard Teensy 4 LED, as per hardware
#define ANATHROTTLE 14   //Analog throttle input
#define ANASPARE 15   //


/*
 * RTD variables
 */
 bool isRTD = 0;

void setup() {
  delay(100);

  //Set up input and output pins
  pinMode(BRAKESW,INPUT);
  pinMode(RTDGATE,OUTPUT);
  pinMode(RTDBUTTON,INPUT);
  pinMode(TEENSY_LED,OUTPUT);
  //digitalWrite(TEENSY_LED,HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  
    isRTD = false;
    disableRTD(&isRTD);
    digitalWrite(TEENSY_LED,LOW);
    delay(2000);
  //Simple RTD check routine
  if (checkRTD())
      {
        enableRTD(&isRTD);
        digitalWrite(TEENSY_LED,HIGH);
        delay(3000);
      }
}

//RTD helper functions

/*
 * bool checkRTD(void): returns true if the brake and button are pressed (high signal), else returns false
 */
bool checkRTD()
{
  if (digitalRead(RTDBUTTON) && digitalRead(BRAKESW))
    return true;
  return false;
}

/*
 * enableRTD(bool *rtdState): enables the RTD gate output
 * params: pointer to the RTD state boolean
 */
void enableRTD(bool *rtdState)
{
  *rtdState = true;
  digitalWrite(RTDGATE,HIGH);
}

/*
 * disableRTD(): disables the RTD gate output
 */

void disableRTD(bool *rtdState)
{
  *rtdState = false;
  digitalWrite(RTDGATE,LOW);
}






 
