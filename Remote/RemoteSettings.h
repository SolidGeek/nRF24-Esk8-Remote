#ifndef _REMOTE_SETTINGS_h
#define _REMOTE_SETTINGS_h

#include <Arduino.h>
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
	bool autoTurnoff;
	uint8_t autoTurnoffTime;


	/* Settings functions */

	RemoteSettings( void );

	void initiate( Remote * _pointer );

	void setDefaultSettings( void );

	void loadSettings( void );

	void saveSettings( void );

private:

	/* Reference to the main object */
	Remote * pointer;

};

#endif
