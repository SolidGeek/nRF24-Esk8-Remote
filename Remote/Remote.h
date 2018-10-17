#ifndef _REMOTE_h
#define _REMOTE_h

/* Including Arduino.h for basic functionality and Wire.h for SPI com */
#include <Arduino.h>
#include <Wire.h>
#include <RF24.h>

#include "constants.h"
#include "RemoteDisplay.h"
#include "RemoteSettings.h"

class Remote
{

public: 

	struct package {		// | Normal 	| Setting 	| Acknowledgement
		uint8_t type;		// | 0 			| 1			| 2
		uint16_t throttle;	// | Throttle 	| Number    | 0
		uint64_t payload;	// | Verify		| Value		| Value
	};

	RemoteDisplay Display;
	RemoteSettings Settings;

	Remote( void );

	void begin( void );

	bool upperTrigger( void );

	bool lowerTrigger( void );

	bool usbConnected( void );

	uint8_t batteryPercentage( void );

  void calculateThrottle( void );
  
	uint16_t getThrottle( void );

	bool transmit( uint8_t type, uint16_t value, uint64_t payload );

  const uint16_t startupTimer = 2000;

private:

	uint16_t hallOutput;
  uint16_t hallRaw;
	uint16_t throttle;

	float voltage;

	uint32_t lastTransmission;

	void measureVoltage( void );

	void measureHallOutput( void );

	int EMA( int newSample, int oldSample, float alpha );

};


#endif
