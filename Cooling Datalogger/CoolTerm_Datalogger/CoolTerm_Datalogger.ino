//NURacing Matthew Whitworth
//datalogger to interface with CoolTerm
//modify freq to control sampling rate
//HX711 library interfaces with load cell amps
//flow rate sensor based on pulse frequency from hall effect

#include <HX711.h>

//calibration factors obtained in seperate sketch
#define calibration_factor_1 481000
#define calibration_factor_2 485000
#define calibration_factor_3 484000

#define LOADCELL_DOUT_PIN_1  8
#define LOADCELL_SCK_PIN_1  3
#define LOADCELL_DOUT_PIN_2  4
#define LOADCELL_SCK_PIN_2  5
#define LOADCELL_DOUT_PIN_3  6
#define LOADCELL_SCK_PIN_3  7

HX711 scale1;
HX711 scale2;
HX711 scale3;

//Flow rate (L/min) = pulse frequency (Hz) / 7.5
volatile int  flow_frequency;  // Measures flow meter pulses
unsigned char flowPin = 2;

long freq = 5.0;  //Number of samples per second (hz)
long t = 0;
int sampleRate;

void flow ()                  // Interrupt function
{ 
   flow_frequency++;
} 

void setup() {
  Serial.begin(9600);
  sampleRate = 1000/freq;

  pinMode(flowPin, INPUT);
  attachInterrupt(0, flow, RISING); // Setup Interrupt
  sei();                            // Enable interrupts   
  
  scale1.begin(LOADCELL_DOUT_PIN_1, LOADCELL_SCK_PIN_1);
  scale1.set_scale(calibration_factor_1);
  scale2.begin(LOADCELL_DOUT_PIN_2, LOADCELL_SCK_PIN_2);
  scale2.set_scale(calibration_factor_2);
  scale3.begin(LOADCELL_DOUT_PIN_3, LOADCELL_SCK_PIN_3);
  scale3.set_scale(calibration_factor_3);
  scale1.tare();  //Assuming there is no weight on the scale at start up, reset the scale to 0
  scale2.tare();  
  scale3.tare();
}

void loop() {
  if(millis() - t >= sampleRate) {
    t = millis();
    //multiply flow_frequency by freq to account for the shorter amount of time to count pulses
    Serial.print((flow_frequency * freq) / 7.5);   // print litres/min
    flow_frequency = 0;                                       // Reset Counter
    Serial.print(",");
    Serial.print(scale1.get_units(), 2); //scale.get_units() returns a float
    Serial.print(",");
    Serial.print(scale2.get_units(), 2); //scale.get_units() returns a float
    Serial.print(",");
    Serial.print(scale3.get_units(), 2); //scale.get_units() returns a float
    Serial.println();
  }
}
