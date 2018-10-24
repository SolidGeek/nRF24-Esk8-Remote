#include "Remote.h"
#include "RemoteDisplay.h"

RemoteDisplay::RemoteDisplay( void ){}

void RemoteDisplay::init( Remote * _pointer )
{
	pointer = _pointer;
 
  /* Defining the type of display used (128x32) */
  u8g2 = new U8G2_SSD1306_128X32_UNIVISION_2_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
}

void RemoteDisplay::begin(){
  u8g2->begin();
}


void RemoteDisplay::update()
{
  if( (millis() - lastUpdate) > updateTimer )
  {
    u8g2->firstPage();

    unsigned long start = millis();
    
    do{

      if( millis() < pointer->startupTimer){
        showStartup();
      }else{

        if(pointer->changeSettings == true)
        {
          // Show settings menu  
          showSettings();
        }
        else if( pointer->usbConnected() )
        {
          showCharging();
        }
        else{
          showTelemetry();
          showThrottle();
          showConnection();
          showRemoteBattery();
        }
      }
      
    }while ( u8g2->nextPage() );

    unsigned long end = millis();
    unsigned long delta = end - start;
    Serial.print("Time: ");
    Serial.println(delta);
    

    lastUpdate = millis();
  }
}

void RemoteDisplay::showSettings( void ) {

  uint8_t x = 0; uint8_t y = 10;

  uint8_t settingNumber = pointer->Settings.currentSetting;

  // Draw setting title
  u8g2->setFont( u8g2_font_profont12_tr );
  u8g2->drawStr( x, y, SETTING_TITLES[ settingNumber ] );

  int val = pointer->Settings.getValue(settingNumber);
  sprintf( buf, "%d", val );

  u8g2->setFont( u8g2_font_10x20_tr );

  if ( pointer->Settings.selected == true )
  {
    u8g2->drawStr(x + 10, y + 20, buf);
  }
  else
  {
    u8g2->drawStr(x, y + 20, buf);
  }  
}

void RemoteDisplay::showStartup( void )
{
  uint8_t x = 16; uint8_t y = 2;
  
  // Draw icon
  u8g2->drawXBMP( x, y, 23, 23, firefly_icon );

  // Draw name
  u8g2->setFont( u8g2_font_fub14_tr );
  u8g2->drawStr( x+32,y+18, "Firefly" );
  
}

void RemoteDisplay::showThrottle( void )
{

  // uint8_t x = 1; uint8_t y = 22;

  uint8_t x = 2; uint8_t y = 0;
  
  uint16_t throttle = pointer->getThrottle();
  uint8_t width;

  if (throttle > THROTTLE_CENTER) {

    width = map(throttle, THROTTLE_CENTER+1, 1023, 0, 80);

    u8g2->drawBox(x, y, width, 2);
    
  } else if (throttle < THROTTLE_CENTER){

    width = map(throttle, 0, THROTTLE_CENTER-1, 80, 0);
    
    u8g2->drawBox(x + 80 - width, y, width, 2);

  }

  /*u8g2->drawRFrame(x, y, 53, 10, 4);

  if (throttle > THROTTLE_CENTER) {

    width = map(throttle, THROTTLE_CENTER+1, 1023, 0, 49);

    if(width > 3){
      u8g2->drawRBox(x + 2, y + 2, width, 6, 2);
    }else{
      // Min width for a Rbox with radius two is 4px
      u8g2->drawRBox(x + 2, y + 2, 4, 6, 2);  
    }
    
  } else if (throttle < THROTTLE_CENTER){

    width = map(throttle, 0, THROTTLE_CENTER-1, 49, 0);

    if(width > 3){
      u8g2->drawRBox(x + 51 - width, y + 2, width, 6, 2);
    }else{
      // Min width for a Rbox with radius two is 4px
      u8g2->drawRBox(x + 47, y + 2, 4, 6, 2);  
    }
  }*/
}

void RemoteDisplay::showConnection() {

  uint8_t x = 88; uint8_t y = 2;
  
  u8g2->drawXBMP(x, y, 12, 12, connection_icon );
    
}

void RemoteDisplay::showTelemetry( void )
{

  uint8_t x = 0; uint8_t y = 31;
  
  // Battery voltage and distance
  u8g2->setFont(u8g2_font_profont12_tr);
  u8g2->drawStr(x + 2, y - 14, "98.4 %");
  u8g2->drawStr(x + 2, y, "8.3 KM");
  // Decimals and speed unit
  u8g2->drawStr(x + 84, y, ".35 KMH" );

  // Speed
  u8g2->setFont(u8g2_font_logisoso22_tn);
  u8g2->drawStr(x + 55, y - 1, "34");

 
}

void RemoteDisplay::showRemoteBattery() {

  uint8_t x =108; uint8_t y = 3;
  uint8_t width;
  
  uint8_t percentage = pointer->batteryPercentage();

  u8g2->drawRFrame(x + 1, y, 16, 8, 2);
  u8g2->drawBox(x, y + 2, 1, 4);

  width = map(percentage, 0, 100, 0, 12);
  u8g2->drawBox(x + 15 - width, y + 2, width, 4);
  
  /*for (uint8_t i = 0; i < 5; i++) {
    uint8_t p = round((100 / 5) * i);
    if (p <= percentage)
    {
      
    }
  }*/
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
	u8g2->drawXBMP(x+11, y+6, 20, 8, charging_icon);
	
	// Draw end of battery
	u8g2->drawRFrame(x+4, y+1, 34, 18, 2);
	u8g2->drawRFrame(x+3, y, 36, 20, 3);
	
	// Draw tip of battery
	u8g2->drawRFrame(x+1, y+7, 3, 6, 0);
	u8g2->drawRFrame(x, y+6, 5, 8, 1);
	
}
