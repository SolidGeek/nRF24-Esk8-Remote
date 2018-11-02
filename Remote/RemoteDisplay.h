#ifndef _REMOTE_DISPLAY_h
#define _REMOTE_DISPLAY_h

/* Including the u8g2 library for the OLED display */
#include <Arduino.h>
#include <U8g2lib.h>
#include "constants.h"

class RemoteDisplay
{
public:

  RemoteDisplay( void );

	void init( Remote * _pointer );

  void begin( void );
  
	void update( void );

	void showCharging( void );

  void showThrottle( void ); 
  
  void showTelemetry( void );

  void showStartup( void );

  void showSettings ( void );

  void showRemoteBattery( void );

  void showConnection( void );

private:
  
  char buf[20];

	/* Reference to the main object */
	Remote * pointer;

  const uint16_t updateTimer = 50; // 20 Hz refresh rate
  uint32_t lastUpdate;

  

	/* Reference to the OLED display */
  U8G2_SSD1306_128X32_UNIVISION_2_HW_I2C * u8g2;

};

#endif
