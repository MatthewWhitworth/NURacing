 /*
	interface.ino
	helper functions for the user interface
	*/

void lostCANL()
{
	fill_solid(leds,3,CRGB::Blue);		//Battery LEDs
}


void updateBatL(uint8_t batState)
	{
	//Defult each LED to green 
	for (int i=0;i<3;i++)
		leds[BAT_L_INDEX + i] = CRGB(0,0,0);
	//Update LED 0
	if (batState <= BAT_CHARGE_SCALE)
	{
		leds[BAT_L_INDEX] = CRGB::Red;
		return;
	}else if (batState <= 2*BAT_CHARGE_SCALE)
	{
		leds[BAT_L_INDEX] = CRGB::Yellow;
		return;
	}else
		leds[BAT_L_INDEX] = CRGB::Green;
	if (batState <= 3*BAT_CHARGE_SCALE)
		return;
	
	//Update LED 1
	if (batState <= 4*BAT_CHARGE_SCALE)
	{
		leds[BAT_L_INDEX+1] = CRGB::Red;
		return;
	}else if (batState <= 5*BAT_CHARGE_SCALE)
	{
		leds[BAT_L_INDEX+1] = CRGB::Yellow;
		return;
	}else
		leds[BAT_L_INDEX+1] = CRGB::Green;
	if (batState <= 6*BAT_CHARGE_SCALE)
		return;
	
	//Update LED 2
	if (batState <= 7*BAT_CHARGE_SCALE)
	{
		leds[BAT_L_INDEX+2] = CRGB::Red;
		return;
	}else if (batState <= 8*BAT_CHARGE_SCALE)
	{
		leds[BAT_L_INDEX+2] = CRGB::Yellow;
		return;
	}else
		leds[BAT_L_INDEX+2] = CRGB::Green;
}
 
void setHardFaultL(uint8_t LEDno)
{
	leds[HF_L_INDEX] = CRGB::Red;
}
 
void clearHardFaultL()
{
	 for (int i=0;i<4;i++)
		 leds[HF_L_INDEX + 2*i] = 0x000800;
}
 
void updateSDL(uint8_t sdState)
{
	if (sdState == 255)
	{
		leds[SD_L_INDEX] = CRGB::Green;
	}else if (sdState == 255)	//TODO change this to reflect precharge state
	{
		leds[SD_L_INDEX] = CRGB::Yellow;
	}else
	{
		leds[SD_L_INDEX] = CRGB::Red;
	}
}
//Update the shutdown indicator LED
//Params sdState the status word of the Shutdown circuit
//Sets light to red if off, orange if precharge, green if active
//Maybe mirror TSAL behaviour instead?
 
void updateMotorTempL(uint8_t tempIn)
{
	
}
//Update the tempreature LED based on tempIN vs motor_warn, motor_danger

void updateRTDL(bool active)
{
	if (active == true)
		leds[RTD_L_INDEX] = CRGB::Green;
	else
		leds[RTD_L_INDEX] = CRGB::Red;
}

void updateTrailBrakeL(bool active)
{
	if (active)
	/* {
		if (millis() % 500 < 250)
			leds[TRAIL_L_INDEX] = CRGB::Red;
		else
			leds[TRAIL_L_INDEX] = CRGB::Black;
	} */
		leds[TRAIL_L_INDEX] = CRGB::Green;
	else
		leds[TRAIL_L_INDEX] = CRGB::Red;
}