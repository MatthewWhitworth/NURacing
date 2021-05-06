/*
	rtd.ino
	Helper functions for the RTD
*/
#include "defines.h"

/*
 * bool checkRTD(void): returns true if the brake and button are pressed (high signal), else returns false
 */
bool checkRTD()
{
  if (digitalRead(RTDBUTTON) == LOW && digitalRead(BRAKESW))
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
  digitalWrite(TEENSY_LED,HIGH);
}

/*
 * disableRTD(): disables the RTD gate output
 */

void disableRTD(bool *rtdState)
{
  *rtdState = false;
  digitalWrite(RTDGATE,LOW);
  digitalWrite(TEENSY_LED,LOW);
}