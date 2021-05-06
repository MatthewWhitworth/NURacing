/*
helper functions for CAN comms
*/

/*Message initialisation
params can message ptr, message length uint8_t
*/

void initCanMsg(CAN_message_t *msg, uint8_t length, int addr)
{
	msg->id = addr;
	msg->len = length;
	msg->flags.extended = 1;
	msg->flags.remote   = 0;
	msg->flags.overrun  = 0;
	msg->flags.reserved = 0;
	for (int i=0;i<length;i++)
  {
	  msg->buf[i] = 0;	//Init the message to zeros
  }
}

/*
 * int collectInputs()
 * Reads the digital IO ports defined at start of program, stores them bitwise in a (single byte unsigned) int
	This is usedd to prepare data for CAN transmission
 */

uint8_t collectInputs()
{
  uint8_t inputs = 0;
  if (digitalRead(SDINERTIA) == HIGH)
    inputs += 1;
  if (digitalRead(SDBOTS) == HIGH)
    inputs += 2;
  if (digitalRead(SDDASH) == HIGH)
    inputs += 4;
  if (digitalRead(APPSTRAIL) == HIGH)
    inputs += 8;
  if (digitalRead(APPSBOUND) == HIGH)
    inputs += 16;
  if (digitalRead(APPSDIS) == HIGH)
    inputs += 32;
  if (digitalRead(BRAKESW) == HIGH)
    inputs += 64;
  if (digitalRead(RTDGATE) == HIGH)
    inputs += 128;
  return inputs;
}