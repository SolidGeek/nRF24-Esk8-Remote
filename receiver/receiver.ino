#include <SPI.h>
#include <EEPROM.h>
#include <nRF24L01.h>
#include "RF24.h"
#include "VescUart.h"

#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.println ((String)millis() + ": " + x)
  #include "printf.h"
#else
  #define DEBUG_PRINT(x)
#endif

// Transmit and receive package
struct package {    // | Normal    | Setting 	| Dummy
	uint8_t type;     // | 0         | 1			  | 2
	uint8_t throttle;	// | Throttle  |         	| 
	uint8_t trigger;	// | Trigger   |        	| 
};

// When receiving a "type: 1" package save the next transmission (a new setting) in this struct 
struct settingPackage {
	uint8_t setting;
	uint64_t value; 
};

// Defining struct to handle callback data (auto ack)
struct callback {
	float ampHours;
	float inpVoltage;
	long rpm;
	long tachometerAbs;
};

// Defining struct to handle receiver settings
struct settings {
	uint8_t triggerMode; // Trigger mode
	uint8_t controlMode; // PWM, PWM & UART or UART only
	uint64_t address;    // Listen on this address
};

const uint8_t numOfSettings = 3;
// Setting rules format: default, min, max.
const short settingRules[numOfSettings][3] {
	{0, 0, 2}, // 0: Killswitch | 1: Cruise       | 2: Data toggle
	{1,	0, 2}, // 0: PPM only   | 1: PPM and UART | 2: UART only
	{-1, 0, 0} // No validation for address in this manner 
};

struct bldcMeasure uartData;
struct callback returnData;
struct package remPackage;
struct settingPackage setPackage;
struct settings rxSettings;

// Define default 8 byte address
const uint64_t defaultAddress = 0xE8E8F0F0E1LL;
const uint8_t defaultChannel = 108;
uint32_t timeoutTimer = 0;
bool recievedData = false;

// Current mode of receiver - 0: Listening | 1: Connected | 2: UPDATING settings
uint8_t statusMode = 0;
#define LISTENING 0
#define UPDATING 1

// Last time data was pulled from VESC
unsigned long lastUartPull;

const int defaultThrottle = 127;
const short timeoutMax = 500;

// Defining receiver pins
const uint8_t CE = 9;
const uint8_t CS = 10;
const uint8_t throttlePin = 5;
const uint8_t resetButtonPin = 6;

// Initiate RF24 class 
RF24 radio(CE, CS);

void setup()
{
	setDefaultEEPROMSettings();
	loadEEPROMSettings();

	#ifdef DEBUG
		Serial.begin(9600);
		DEBUG_PRINT("Booting");
		printLoadedSettings();
	#else
		// Using RX and TX to get VESC data
		SERIALIO.begin(115200);
	#endif

	radio.begin();
	// radio.setChannel(defaultChannel);
	radio.enableAckPayload();
	radio.enableDynamicPayloads();
	radio.openReadingPipe(1, rxSettings.address);
	radio.startListening();

	pinMode(throttlePin, OUTPUT);
	pinMode(resetButtonPin, INPUT_PULLUP);

	// Set default throttle in startup
	analogWrite(throttlePin, defaultThrottle);
}

int dump;

void loop()
{
	// If transmission is available
	while (radio.available())
	{
		// Read and store the received package
		radio.read( &remPackage, sizeof(remPackage) );

    if( remPackage.type <= 2 ){
      DEBUG_PRINT("Received new package: '" + (String)remPackage.type + "-" + (String)remPackage.throttle + "-" + (String)remPackage.trigger + "'" );
      timeoutTimer = millis();
      recievedData = true;
    }
	}

	// New data received
	if(recievedData == true){

		if ( remPackage.type == 0 ) {
			// Normal package
			updateThrottle( remPackage.throttle );
			
			if( rxSettings.controlMode != 0 ){
				getUartData();
			}

			// The next time a transmission is received, the returnData will be sent back in acknowledgement 
			radio.writeAckPayload(rxSettings.address, &returnData, sizeof(returnData));

		} else if ( remPackage.type == 1 ) {

			// Next package will be a change of setting
			
			waitForSetting();
		}
	
		recievedData = false;
	}

	if ( timeoutMax <= ( millis() - timeoutTimer ) )
	{
	    // No speed is received within the timeout limit.
	    updateThrottle( defaultThrottle );
	    DEBUG_PRINT("Timeout");
      timeoutTimer = millis();
     
	}
}



void waitForSetting(){

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
    radio.writeAckPayload(rxSettings.address, &value, sizeof(value));
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
    }

    delay(500);
  }

  if (receivedSetting == false || receivedConfirm == false)
  {
    DEBUG_PRINT("Failed! Clearing receiver buffer");
    delay(500);
    while (radio.available())
    {
      radio.read( &setPackage, sizeof(setPackage) );
      DEBUG_PRINT("Cleared buffer");
    }
  }
}

// Restart the nrf24 after receiving new address
void restartReceiver(){
  DEBUG_PRINT("Resetting receiver address");
  radio.stopListening();
  radio.closeReadingPipe(1);

  delay(50);
  radio.openReadingPipe(1, rxSettings.address);
  radio.startListening();
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

  if(setting == 2) {
    restartReceiver(); 
  }
}

void updateThrottle( uint8_t throttle )
{
	switch ( rxSettings.controlMode )
	{
		// PPM
		case 0:
			// Write the PWM signal to the ESC (0-255).
    		analogWrite(throttlePin, throttle);
		break;

		// PPM and UART
		case 1: 
			// Write the PWM signal to the ESC (0-255).
    		analogWrite(throttlePin, throttle);
		break;

		 // UART
		case 2:
			// Update throttle with UART
		break;
	}
} 

void getUartData()
{
	if ( millis() - lastUartPull >= 250 ) {

		lastUartPull = millis();

		// Only get what we need
		if ( VescUartGetValue(uartData) )
		{
			returnData.ampHours 		  = uartData.ampHours;
			returnData.inpVoltage 	  = uartData.inpVoltage;
			returnData.rpm 				    = uartData.rpm;
			returnData.tachometerAbs 	= uartData.tachometerAbs;
		} 
		else
		{
			returnData.ampHours 		  = 0.0;
			returnData.inpVoltage 		= 0.0;
			returnData.rpm 				    = 0;
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
}

// Write settings to the EEPROM then exiting settings menu.
void updateEEPROMSettings()
{
	EEPROM.put(0, rxSettings);
}

void printLoadedSettings()
{
	for(int i = 0; i < numOfSettings; i++){
		if(i == 2){
			DEBUG_PRINT( uint64ToString( rxSettings.address ) );
		}else{
			DEBUG_PRINT( (String)getSettingValue(i) );
		}
	}
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
