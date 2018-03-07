#include <SPI.h>
#include <EEPROM.h>
#include <nRF24L01.h>
#include "RF24.h"
#include "VescUart.h"

#define DEBUG

#ifdef DEBUG
	#define DEBUG_PRINT(x)  Serial.println (x)
	#include "printf.h"
#else
	#define DEBUG_PRINT(x)
#endif

// Transmit and receive package
struct package {		// | Normal    | Setting 	| Confirm
	uint8_t type;		  // | 0         | 1			  | 2
	uint8_t throttle;	// | Throttle  | ---      | ---
	uint8_t trigger;	// | Trigger   | ---      | ---
} remPackage;

#define NORMAL 0
#define SETTING 1
#define CONFIRM 2

// When receiving a "type: 1" package save the next transmission (a new setting) in this struct 
struct settingPackage {
	uint8_t setting;
	uint64_t value; 
} setPackage;

// Defining struct to handle callback data (auto ack)
struct callback {
	float ampHours;
	float inpVoltage;
	long rpm;
	long tachometerAbs;
} returnData;

// Defining struct to handle receiver settings
struct settings {
	uint8_t triggerMode; // Trigger mode
	uint8_t controlMode; // PWM, PWM & UART or UART only
	uint8_t motorPoles; // Pole pairs
	uint64_t address;    // Listen on this address
} rxSettings;

// Struct used to store UART data
struct bldcMeasure uartData;

const uint8_t numOfSettings = 3;
// Setting rules format: default, min, max.
const short settingRules[numOfSettings][3] {
	{0, 0, 1}, // 0: Killswitch | 1: Cruise   
	{1,	0, 2}, // 0: PPM only   | 1: PPM and UART | 2: UART only
	{-1, 0, 0} // No validation for address in this manner 
};

// Define default 8 byte address
const uint64_t defaultAddress = 0xE8E8F0F0E1LL;
const uint8_t defaultChannel = 108;
uint32_t timeoutTimer = 0;
bool recievedData = false;

// Current mode of receiver - 0: Connected | 1: Timeout | 2: Updating settings
#define CONNECTED 0
#define TIMEOUT 1
#define COMPLETE 2
#define FAILED 3

// Last time data was pulled from VESC
unsigned long lastUartPull;

// Variables for cruise-control
long cruiseRPM;
long lastRPM; 
uint8_t cruiseThrottle;
bool cruising;

// Address reset button
unsigned long resetButtonTimer;
bool resetButtonState = LOW;

// Status blink LED
unsigned long lastStatusBlink;
bool statusBlinkFlag = LOW;

const uint8_t defaultThrottle = 127;
const short timeoutMax = 500;

// Defining receiver pins
const uint8_t CE = 9;
const uint8_t CS = 10;
const uint8_t statusLedPin = 4;
const uint8_t throttlePin = 5;
const uint8_t resetAddressPin = 6;

// Initiate RF24 class 
RF24 radio(CE, CS);

void setup()
{
	// setDefaultEEPROMSettings(); // Use this first time you upload code

	#ifdef DEBUG
		Serial.begin(115200);
		DEBUG_PRINT("** Esk8-remote receiver **");
		printf_begin();
	#else
		// Using RX and TX to get VESC data
		SetSerialPort(&Serial);
		Serial.begin(115200);
	#endif

	loadEEPROMSettings();

	initiateReceiver();

	pinMode(throttlePin, OUTPUT);
	pinMode(statusLedPin, OUTPUT);
	pinMode(resetAddressPin, INPUT_PULLUP);

	analogWrite(throttlePin, defaultThrottle);

	DEBUG_PRINT("Setup complete - begin listening");

}

void loop()
{ 
	/* Begin address reset */
	if (digitalRead(resetAddressPin) == LOW) {
		if(resetButtonState == LOW){
			resetButtonTimer = millis();
		}
		resetButtonState = HIGH;
	}else{
		resetButtonState = LOW;
	}

	if (resetButtonState == HIGH && (millis() - resetButtonTimer) > 5000 ) {

		DEBUG_PRINT("Loading default address");

		// Load default address
		rxSettings.address = defaultAddress;
		updateEEPROMSettings(); 

		// Reinitiate the recevier module
		initiateReceiver();

		statusBlink(COMPLETE);

		resetButtonTimer = millis();
	}
	/* End address reset */

	/* Begin listen for transmission */
	while (radio.available() && !recievedData)
	{
		// Read and store the received package
		radio.read( &remPackage, sizeof(remPackage) );
		DEBUG_PRINT( uint64ToAddress(rxSettings.address) + " - New package: '" + (String)remPackage.type + "-" + (String)remPackage.throttle + "-" + (String)remPackage.trigger + "'" );

		if( remPackage.type <= 2 ){
			timeoutTimer = millis();
			recievedData = true;
		}
	}
	/* End listen for transmission */

	/* Begin data handling */
	if(recievedData == true){

		statusBlink(CONNECTED);

		if ( remPackage.type == NORMAL ) {

			// Normal package
			controlThrottle( remPackage.throttle, remPackage.trigger );
			
			if( rxSettings.controlMode != 0 ){
				#ifdef DEBUG
					DEBUG_PRINT("Getting VESC data");
				#else
					getUartData();
				#endif
			}

			// The next time a transmission is received, the returnData will be sent back in acknowledgement 
			radio.writeAckPayload(1, &returnData, sizeof(returnData));

		} else if ( remPackage.type == SETTING ) {

			// Next package will be a change of setting
			acquireSetting();

		}
	
		recievedData = false;
	}
	/* End data handling */

	/* Begin timeout handling */
	if ( timeoutMax <= ( millis() - timeoutTimer ) )
	{
		// No speed is received within the timeout limit.
		statusBlink(TIMEOUT);
		controlThrottle( defaultThrottle, false );
		timeoutTimer = millis();

		DEBUG_PRINT( uint64ToAddress(rxSettings.address) + " - Timeout");
	}
	/* End timeout handling */
}

void statusBlink(uint8_t statusCode){
	
	short ontime, offtime;

	switch(statusCode){
		case CONNECTED:
			ontime = 500; offtime = 0;
		break;

		case TIMEOUT:
			ontime = 300; offtime = 300;
		break;

		case COMPLETE:
			ontime = 50; offtime = 50;

			for(uint8_t i = 0; i < 6; i++){
				digitalWrite(statusLedPin, statusBlinkFlag);
				statusBlinkFlag = !statusBlinkFlag;

				if(statusBlinkFlag){
					delay(ontime);
				}else{
					delay(offtime);  
				}
			}

			statusBlinkFlag = LOW;

			return;

		break;

		case FAILED:
			ontime = 500; offtime = 200;

			digitalWrite(statusLedPin, LOW);

			delay(1000);

			for(uint8_t i = 0; i < 6; i++){
				digitalWrite(statusLedPin, statusBlinkFlag);
				statusBlinkFlag = !statusBlinkFlag;

				if(statusBlinkFlag){
					delay(ontime);
				}else{
					delay(offtime);  
				}
			}

			statusBlinkFlag = LOW;

			return;
		break;
	}

	if ((millis() - lastStatusBlink) > offtime && statusBlinkFlag == LOW) {
		statusBlinkFlag = HIGH;
		lastStatusBlink = millis();
	}
	else if ((millis() - lastStatusBlink) > ontime && statusBlinkFlag == HIGH) {
		statusBlinkFlag = LOW;
		lastStatusBlink = millis();
	}
	if ( offtime == 0 ){
		statusBlinkFlag = HIGH;  
	}

	digitalWrite(statusLedPin, statusBlinkFlag);
}

void acquireSetting() {
  
  uint8_t setting;
  uint64_t value;
  
  unsigned long beginTime = millis();
  
  bool receivedSetting = false;
  bool receivedConfirm = false;
  
  DEBUG_PRINT("Waiting for new setting...");
  
  // Wait for new setting
  while ( receivedSetting == false && 500 >= ( millis() - beginTime) ) {

    if ( radio.available() ) {
      
      // Read and store the received setting
      radio.read( &setPackage, sizeof(setPackage));

      if(receivedSetting == false){
        DEBUG_PRINT("Received new setting");
        setting = setPackage.setting;
        value = setPackage.value;
        
        // Return the setPackage in acknowlegdement
        radio.writeAckPayload(1, &setPackage, sizeof(setPackage));
      }
      
      receivedSetting = true;

      delay(100);

    }
  }

  // Clear receiver buffer
  beginTime = millis();
  while ( radio.available() && 500 >= ( millis() - beginTime) ) {
    DEBUG_PRINT("Cleared");
    radio.read( &setPackage, sizeof(setPackage) );
    delay(100);
  }

  if (receivedSetting == true) {

    // Check if the TX Ack DATA is matching
    DEBUG_PRINT("Waiting for confirmation");

    beginTime = millis();

    while (1000 >= ( millis() - beginTime) && !receivedConfirm) {

      if( radio.available() ){

        radio.read( &remPackage, sizeof(remPackage));

        DEBUG_PRINT(String(remPackage.type));

        if(remPackage.type == CONFIRM){
          receivedConfirm = true;
          DEBUG_PRINT("Confirmed");
        }
      }

      delay(100);
        
    }

    if( receivedConfirm == true){
      updateSetting(setting, value);
      DEBUG_PRINT("Updated setting.");

      statusBlink(COMPLETE);
    }

    delay(100);

  }

  // Something went wrong, lets clear all buffers

  if (receivedSetting == false || receivedConfirm == false || radio.available()) {

    DEBUG_PRINT("Failed! Clearing buffer");
    statusBlink(FAILED);

    beginTime = millis();

    while (radio.available() && 500 >= ( millis() - beginTime)) {

      radio.read( &setPackage, sizeof(setPackage) );
      radio.read( &remPackage, sizeof(remPackage) );

      DEBUG_PRINT("Cleared buffer");
    }
  }

}

/*
void acquireSetting(){

	uint64_t value;
	unsigned long beginTime = millis(); 
	bool receivedSetting = false;
	bool receivedConfirm = false;

	DEBUG_PRINT("Waiting for new setting...");

	// Wait for new setting
	while(500 >= ( millis() - beginTime) && receivedSetting == false){

		while (radio.available())
		{
			// Read and store the received
			radio.read( &setPackage, sizeof(setPackage) );
			receivedSetting = true;

			if(setPackage.setting == 11){
				DEBUG_PRINT("Received new setting: '" + (String)setPackage.setting + "=" + uint64ToAddress(setPackage.value) + "'");  
			}else{
				DEBUG_PRINT("Received new setting: '" + (String)setPackage.setting + "=" + uint64ToString(setPackage.value) + "'");
			}
		}
	}

	if ( receivedSetting == true ) {

		value = setPackage.value;

		// Return the new setting value with auto ack to validate the process
		radio.writeAckPayload(1, &value, sizeof(value));
		DEBUG_PRINT("Queued setting acknowledgement");

		beginTime = millis(); 

		// Wait for dummy package (otherwise acknowledgement will not be send)
		while(500 >= ( millis() - beginTime) && receivedConfirm == false){

			while (radio.available())
			{
				radio.read( &remPackage, sizeof(remPackage) );
				receivedConfirm = true;

				DEBUG_PRINT("Received confirm package.");
			}
		}

		if( receivedConfirm == true ){
			updateSetting(setPackage.setting, value);
			DEBUG_PRINT("Updated setting.");
			
			statusBlink(COMPLETE);
		}

		delay(200);
	}

	if (receivedSetting == false || receivedConfirm == false)
	{
		DEBUG_PRINT("Failed! Clearing receiver buffer");

		statusBlink(FAILED);

		delay(200);
		while (radio.available())
		{
			radio.read( &setPackage, sizeof(setPackage) );
			DEBUG_PRINT("Cleared buffer");
		}
	}
}*/

void initiateReceiver(){

	radio.begin();
	radio.setChannel(defaultChannel);
	radio.enableAckPayload();
	radio.enableDynamicPayloads();
	radio.openReadingPipe(1, rxSettings.address);
	radio.startListening();

	#ifdef DEBUG
		DEBUG_PRINT("Printing receiver details");
		radio.printDetails();
	#endif

}

// Update a single setting value
void updateSetting( uint8_t setting, uint64_t value)
{
	// Map remote setting indexes to receiver settings
	switch( setting ){
		case 0: setting = 0; break;  // TriggerMode
		case 7: setting = 1; break;  // ControlMode
		case 11: setting = 2; break; // Address
	}
	
	setSettingValue( setting, value);
	
	updateEEPROMSettings(); 

	// The address has changed, we need to reinitiate the receiver module
	if(setting == 2) {
		initiateReceiver(); 
	}
}

void setThrottle( uint8_t throttle )
{
	analogWrite(throttlePin, throttle);
}

void controlThrottle( uint8_t throttle , bool trigger )
{
	// Kill switch
	if( rxSettings.triggerMode == 0)
	{
		DEBUG_PRINT("Killswitch");
		if ( trigger == false || throttle < 127 )
		{
			setThrottle( throttle );
		}
		else
		{
			setThrottle( defaultThrottle );
		}
	}

	// Cruise control
	else if( rxSettings.triggerMode == 1)
	{ 
		if( trigger == true )
		{

			// If control mode is PPM only
			if( rxSettings.controlMode == 0){
				// Keep current PPM value until trigger is released
				if(cruising == false){
					cruiseThrottle = throttle;
					cruising = true;
				}
				
			}else{

				if(cruising == false){
					cruiseRPM = returnData.rpm;
					cruiseThrottle = throttle;
					cruising = true;
				}

				// While cruise control is active, try to achieve the same ERPM as just measured

				if( lastRPM != returnData.rpm && returnData.rpm != 0) {

					if( returnData.rpm < cruiseRPM ){
						cruiseThrottle += 1;
					}else if ( returnData.rpm > cruiseRPM ){
						cruiseThrottle -= 1;
					}
				}

				lastRPM = returnData.rpm;

			}

			setThrottle( cruiseThrottle );

			#ifdef DEBUG
				DEBUG_PRINT( "Cruise control throttle: " + (String)cruiseThrottle );
			#endif
		}
		else
		{
			setThrottle( throttle );
			cruising = false;
		}
	}
} 

void getUartData()
{
	if ( millis() - lastUartPull >= 200 ) {

		lastUartPull = millis();

		// Only get what we need
		if ( VescUartGetValue(uartData) )
		{
			returnData.ampHours 		= uartData.ampHours;
			returnData.inpVoltage		= uartData.inpVoltage;
			returnData.rpm 				= uartData.rpm;
			returnData.tachometerAbs 	= uartData.tachometerAbs;
		} 
		else
		{
			returnData.ampHours 		= 0.0;
			returnData.inpVoltage 		= 0.0;
			returnData.rpm 				= 0;
			returnData.tachometerAbs 	= 0;
		}
	}
}

String uint64ToString(uint64_t number)
{
	unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
	unsigned long part2 = (unsigned long)((number));

	if(part1 == 0){
		return String(part2, DEC);
	}
	
	return String(part1, DEC) + String(part2, DEC);
}

String uint64ToAddress(uint64_t number)
{
	unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
	unsigned long part2 = (unsigned long)((number));

	return String(part1, HEX) + String(part2, HEX);
}

// Settings functions

void setDefaultEEPROMSettings()
{
	DEBUG_PRINT("Loading default settings.");
	for (int i = 0; i < numOfSettings; i++) {
		setSettingValue(i, settingRules[i][0]);
	}

	rxSettings.address = defaultAddress;
	updateEEPROMSettings();
}

void loadEEPROMSettings()
{
	bool rewriteSettings = false;

	// Load settings from EEPROM to custom struct
	EEPROM.get(0, rxSettings);

	// Loop through all settings to check if everything is fine
	for ( int i = 0; i < numOfSettings; i++ ) {
		int val = getSettingValue(i);

		// If setting default value is -1, don't check if its valid
		if( settingRules[i][0] != -1 )
		{
			if ( !inRange( val, settingRules[i][1], settingRules[i][2] ) )
			{
				// Setting is damaged or never written. Rewrite default.
				rewriteSettings = true;
				setSettingValue(i, settingRules[i][0] );
			}
		}
	}

	if (rewriteSettings == true)
	{
		updateEEPROMSettings();
	}

	DEBUG_PRINT("Settings loaded");
}

// Write settings to the EEPROM
void updateEEPROMSettings()
{
	EEPROM.put(0, rxSettings);
}

// Set a value of a specific setting by index.
void setSettingValue(int index, uint64_t value)
{
	switch (index) {
		case 0: rxSettings.triggerMode = value; break;
		case 1: rxSettings.controlMode = value; break;
		case 2: rxSettings.address = value;     break;
	}
}

// Get settings value by index (usefull when iterating through settings).
int getSettingValue(uint8_t index)
{
	int value;
	switch (index) {
		case 0: value = rxSettings.triggerMode; break;
		case 1: value = rxSettings.controlMode; break;
	}
	return value;
}

bool inRange(int val, int minimum, int maximum)
{
	return ((minimum <= val) && (val <= maximum));
}
