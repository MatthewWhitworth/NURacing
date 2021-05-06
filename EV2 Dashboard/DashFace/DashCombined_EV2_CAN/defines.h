/*
 * defines.h
 */
#ifndef DEFINES_H
#define DEFINES_H

//Disable interrupts during FastLED operations
//#define FASTLED_ALLOW_INTERRUPTS 0
#define FASTLED_INTERRUPT_RETRY_COUNT 0
 /*
 * General IO pin definitions
 */
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
#define BRAKEPRESFRONT 12   //Onboard Teensy 4 LED, as per hardware
#define TEENSY_LED   13   //Onboard Teensy 4 LED, as per hardware
#define ANATHROTTLE 14   //Analog throttle input
#define BRAKEPRESFRONT 15   //

//Defined values for FastLED and the 7 seg 
#define LED_DATA_PIN 20  //FastLED output pin
#define NUM_LEDS 14
#define LED_BRIGHTNESS 255


#define LED_OFFSET 0
#define BAT_L_INDEX 0
#define RTD_L_INDEX 13
#define HF_L_INDEX 4
#define SD_L_INDEX 3
#define TRAIL_L_INDEX 11
//#define MTEMP_L_INDEX 8

#define MOTOR_WARN_TEMP 100
#define MOTOR_DANGER_TEMP 120
#define BAT_CHARGE_SCALE 25		//Divide battery charge by this to obtain a percent value

//RTD Bounce check
#define RTD_TOTALBOUNCE 2000

 
 //CAN parameters
 
#define CAN_BAUD 1000000



#endif
