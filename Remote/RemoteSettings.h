#ifndef _REMOTE_SETTINGS_h
#define _REMOTE_SETTINGS_h

#include <Arduino.h>
#include <EEPROM.h>
#include "constants.h"

class RemoteSettings
{

public:

	/* Settings variables */

	uint8_t mode;
	bool displayRotate;
	uint8_t upperTrigger;
	uint8_t lowerTrigger;
	uint16_t throttleMin;
	uint16_t throttleCenter;
	uint16_t throttleMax;
	uint8_t throttleDeadzone;
	uint8_t throttleLimit;
	uint8_t brakeLimit;
	uint8_t batteryCells;
	uint8_t batteryMinVoltage;
	uint8_t batteryMaxVoltage;
	uint8_t motorPoles;
	uint8_t pulleyMotor;
	uint8_t pulleyWheel;
	uint8_t wheelDiameter;
	uint8_t autoTurnoffTime;

  uint16_t versionNum;

public:
	/* Settings functions */

	RemoteSettings( void );

	void init( Remote * _pointer );

  void setValue( uint8_t index, uint16_t value );

  uint16_t getValue( uint8_t index );

	void reset( void );

	void load( void );

	void save( void );

  bool inRange( uint16_t value, uint8_t index );

  void printSettings( void );


};

#endif
