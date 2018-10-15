#ifndef _REMOTE_h
#define _REMOTE_h

#include <Arduino.h>
#include "RemoteDisplay.h"
#include "RemoteSettings.h"
#include "constants.h"

class Remote{

struct package {		// | Normal 	| Setting 	| Acknowledgement
		uint8_t type;		// | 0 			| 1			| 2
		uint16_t throttle;	// | Throttle 	| Number    | 0
		uint64_t payload;	// | Verify		| Value		| Value
	};

public: 

	RemoteDisplay display;
	RemoteSettings settings;

	Remote(void);

	bool upperTrigger( void );

	bool lowerTrigger( void );

	bool usbConnected( void );

	uint8_t batteryPercentage( void );

	void throttlePosition( void );

	bool transmit( uint8_t type, uint16_t value, uint64_t payload );


private:

	uint16_t hallValue;
	uint16_t throttle;

	float voltage;

	uint32_t lastTransmission;

	void measureVoltage( void );

	void measureHallOutput( void );

	int EMA( int newSample, int oldSample, float alpha );

};


#endif