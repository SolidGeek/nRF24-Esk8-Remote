#ifndef _REMOTE_DISPLAY_h
#define _REMOTE_DISPLAY_h

/* Including the u8g2 library for the OLED display */
#include <Arduino.h>
#include <U8g2lib.h>
#include "constants.h"

class RemoteDisplay
{

public:

	RemoteDisplay(void);

	void initiate( Remote * _pointer );

  void begin( void );
  
	void update( void );

	void showCharging( int percentage );

private:

	/* Reference to the main object */
	Remote * pointer;

	char buf[20];

  U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C * u8g2;

};

#endif
