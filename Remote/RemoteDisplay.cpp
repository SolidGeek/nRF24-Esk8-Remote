#include "Remote.h"
#include "RemoteDisplay.h"

RemoteDisplay::RemoteDisplay( void ){}

void RemoteDisplay::init( Remote * _pointer )
{
	pointer = _pointer;

  /* Defining the type of display used (128x32) */
  u8g2 = new U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
}

void RemoteDisplay::begin(){
  u8g2->begin();
}


void RemoteDisplay::update()
{
  if( (millis() - lastUpdate) > updateTimer )
  {
    u8g2->firstPage();
    
    do{

      if( millis() < pointer->startupTimer)
        showStartup();
      else{

        if( pointer->usbConnected() )
          showCharging();
        else{
          showTelemetry();
        }
      }
      
    }while ( u8g2->nextPage() );

    lastUpdate = millis();
  }
}

void RemoteDisplay::showStartup( void )
{
  uint8_t x = 16; uint8_t y = 2;
  
  // Draw icon
  u8g2->drawXBM(x, y, 23, 23, firefly_icon);

  // Draw name
  u8g2->setFont(u8g2_font_fub14_tr);
  u8g2->drawStr(x+32,y+18, "Firefly");
  
}

void RemoteDisplay::showThrottle( void )
{

  uint8_t x = 0; uint8_t y = 18;
  
  uint8_t width;
  
  // Draw throttle
  u8g2->drawHLine(x, y, 52);
  u8g2->drawVLine(x, y, 10);
  u8g2->drawVLine(x + 52, y, 10);
  u8g2->drawHLine(x, y + 10, 5);
  u8g2->drawHLine(x + 52 - 4, y + 10, 5);

  if (pointer->getThrottle() >= 512) {
    width = map(pointer->getThrottle(), 512, 1023, 0, 49);

    for (uint8_t i = 0; i < width; i++)
    {
      u8g2->drawVLine(x + i + 2, y + 2, 7);
    }
  } else {
    width = map(pointer->getThrottle(), 0, 511, 49, 0);
   
    for (uint8_t i = 0; i < width; i++)
    {
      u8g2->drawVLine(x + 50 - i, y + 2, 7);
    }
  }

}

void RemoteDisplay::showTelemetry( void )
{
  
}

void RemoteDisplay::showCharging( void )
{
	// Base position 
	uint8_t x = 72; uint8_t y = 6;

  int percentage = pointer->batteryPercentage();

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
