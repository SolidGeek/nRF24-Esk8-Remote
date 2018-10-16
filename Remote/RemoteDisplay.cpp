#include "Remote.h"
#include "RemoteDisplay.h"

RemoteDisplay::RemoteDisplay( void )
{
	/* Defining the type of display used (128x32) */
	
}

void RemoteDisplay::initiate( Remote * _pointer )
{
	pointer = _pointer;

  u8g2 = new U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C(U8G2_R0, U8X8_PIN_NONE);

}

void RemoteDisplay::begin(){
  u8g2->begin();
}


void RemoteDisplay::update()
{

  Serial.println( pointer->batteryPercentage() );
  
	u8g2->firstPage();
 
	do {

		showCharging( pointer->batteryPercentage() );
		
	} while ( u8g2->nextPage() );
}

void RemoteDisplay::showCharging( int percentage )
{

	// Base position 
	uint8_t x = 72; uint8_t y = 6;

	// Convert integer to char array and append percentage
	sprintf(buf, "%d", percentage);
	char tmp[2] = "%";
	strcat(buf, tmp);
	
	// Draw battery level
	u8g2->setFont(u8g2_font_fub14_tr);
	u8g2->drawStr(x-62,y+18, buf);
	
	// Draw lightning
	u8g2->drawXBM(x+11, y+6, 20, 8, charging_icon);
	
	// Draw end of battery
	u8g2->drawRFrame(x+4, y+1, 34, 18, 2);
	u8g2->drawRFrame(x+3, y, 36, 20, 3);
	
	// Draw tip of battery
	u8g2->drawRFrame(x+1, y+7, 3, 6, 0);
	u8g2->drawRFrame(x, y+6, 5, 8, 1);
	
}
