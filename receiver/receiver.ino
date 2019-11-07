#include <SPI.h>
#include <EEPROM.h>
#include <Servo.h>
#include "RF24.h"
#include "VescUart.h"

// #define DEBUG

// #define FIREFLYPCB // If receiver is based on the receiver PCB
#define VERSION 2.0

#ifdef DEBUG
	#define DEBUG_PRINT(x)  Serial.println (x)
	#include "printf.h"
#else
	#define DEBUG_PRINT(x)
#endif

// Transmit and receive package
struct package {		  // | Normal 	| Setting 	| Confirm
	uint8_t type;		    // | 0 			  | 1 		    | 2
	uint16_t throttle;	// | Throttle | ---		    | ---
	uint8_t trigger;	  // | Trigger 	| --- 		  | ---
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
	uint64_t address;    // Listen on this address
  float firmVersion;   
} rxSettings;

const uint8_t numOfSettings = 4;
// Setting rules format: default, min, max.
const short settingRules[numOfSettings][3] {
	{0, 0, 1}, // 0: Killswitch | 1: Cruise   
	{1,	0, 2}, // 0: PPM only   | 1: PPM and UART | 2: UART only
	{-1, 0, 0}, // No validation for address in this manner 
  {-1, 0, 0}
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
uint16_t uartPullInterval = 250;

// Cruise control
uint16_t cruiseThrottle;
uint16_t cruiseRPM;
bool cruising;

// Address reset button
unsigned long resetButtonTimer;
bool resetButtonState = LOW;

// Status blink LED
uint8_t statusCode = 0;
bool statusLedState = false;
short statusCycleTime = 0;
unsigned long previousStatusMillis, currentMillis, startCycleMillis = 0;

const uint16_t defaultThrottle = 512;
const short timeoutMax = 500;

// Defining receiver pins
const uint8_t CE = 9;
const uint8_t CS = 10;

#ifdef FIREFLYPCB
  const uint8_t statusLedPin = 13;
#else
  const uint8_t statusLedPin = 6;
#endif

const uint8_t throttlePin = 5;
const uint8_t resetAddressPin = 4;

// Initiate RF24 class 
RF24 radio(CE, CS);

// Initiate Servo class
Servo esc;

// Initiate VescUart class for UART communication
VescUart UART;

void setup()
{
	#ifdef DEBUG
    UART.setDebugPort(&Serial);
		Serial.begin(115200);
		DEBUG_PRINT("** Esk8-remote receiver **");
		printf_begin();
	#else
    #ifndef FIREFLYPCB
		  // Using RX and TX to get VESC data
      UART.setSerialPort(&Serial);
      Serial.begin(115200);
    #endif
	#endif

  #ifdef FIREFLYPCB
    // Uses the Atmega32u4 that has a seperate UART port
    UART.setSerialPort(&Serial1);
    // Uses lower baud rate, since it runs on 8Mhz (115200 baud is to high).
    Serial1.begin(19200);
  #endif

	loadEEPROMSettings();
	initiateReceiver();

	pinMode(statusLedPin, OUTPUT);
	pinMode(resetAddressPin, INPUT_PULLUP);
	esc.attach(throttlePin);

	DEBUG_PRINT("Setup complete - begin listening");
}

void loop()
{ 
  /* Control Status LED */
  controlStatusLed();
  
	/* Begin address reset */
	if (digitalRead(resetAddressPin) == LOW) {
		if(resetButtonState == LOW){
			resetButtonTimer = millis();
		}
		resetButtonState = HIGH;
	}else{
		resetButtonState = LOW;
	}

	if (resetButtonState == HIGH && (millis() - resetButtonTimer) > 3000 ) {

		DEBUG_PRINT("Loading default address");

		// Load default address
		rxSettings.address = defaultAddress;
		updateEEPROMSettings(); 

		// Reinitiate the recevier module
		initiateReceiver();

		setStatus(COMPLETE);

		resetButtonTimer = millis();
	}
	/* End address reset */

	/* Begin listen for transmission */
	while (radio.available() && !recievedData)
	{
		// Read and store the received package
		radio.read( &remPackage, sizeof(remPackage) );
		DEBUG_PRINT( "New package: '" + String(remPackage.type) + "-" + String(remPackage.throttle) + "-" + String(remPackage.trigger) + "'" );
    delay(10); 
    
		if( remPackage.type <= 2 ){
			timeoutTimer = millis();
			recievedData = true;
		}
	}
	/* End listen for transmission */

	/* Begin data handling */
	if(recievedData == true){

		setStatus(CONNECTED);

		if ( remPackage.type == NORMAL ) {

			// Normal package
			speedControl( remPackage.throttle, remPackage.trigger );
			
			if( rxSettings.controlMode != 0 ){
				#ifdef DEBUG
				  DEBUG_PRINT("Getting VESC data");

          #ifdef FIREFLYPCB
            getUartData();
          #endif
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
		setStatus(TIMEOUT);
		speedControl( defaultThrottle, false );
		timeoutTimer = millis();

		DEBUG_PRINT( uint64ToAddress(rxSettings.address) + " - Timeout");
	}
	/* End timeout handling */
}

void setStatus(uint8_t code){

  short cycle = 0;

  switch(code){
    case COMPLETE:  cycle = 500;    break;
    case FAILED:    cycle = 1400;   break;
  }

  currentMillis = millis();

  if(currentMillis - startCycleMillis >= statusCycleTime){
    statusCode = code;
    statusCycleTime = cycle; 
    startCycleMillis = currentMillis;
  }
}

void controlStatusLed(){

  short oninterval, offinterval, cycle;

  switch(statusCode){
    case TIMEOUT:   oninterval = 300;   offinterval = 300;  break;
    case COMPLETE:  oninterval = 50;    offinterval = 50;   break;
    case FAILED:    oninterval = 500;   offinterval = 200;  break;
  }

  currentMillis = millis();

  if (currentMillis - previousStatusMillis >= offinterval && statusLedState == false ) {

    previousStatusMillis = currentMillis;
    statusLedState = !statusLedState;
    
  }else if(currentMillis - previousStatusMillis >= oninterval && statusLedState == true){

    previousStatusMillis = currentMillis;
    statusLedState = !statusLedState;
    
  }

  if(statusCode == CONNECTED){
    analogWrite(statusLedPin, map(remPackage.throttle, 0, 1023, 0, 255)); 
  }else{
    digitalWrite(statusLedPin, statusLedState);
  }  
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

			setStatus(COMPLETE);
		}

		delay(100);
	}

	// Something went wrong, lets clear all buffers

	if (receivedSetting == false || receivedConfirm == false || radio.available()) {

		DEBUG_PRINT("Failed! Clearing buffer");
		setStatus(FAILED);

		beginTime = millis();

		while (radio.available() && 500 >= ( millis() - beginTime)) {

			radio.read( &setPackage, sizeof(setPackage) );
			radio.read( &remPackage, sizeof(remPackage) );

			DEBUG_PRINT("Cleared buffer");
		}
	}
}

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

void setCruise ( bool cruise = true, uint16_t setPoint = defaultThrottle, uint16_t throttle = defaultThrottle ){
  if( rxSettings.controlMode == 0 ){

    setThrottle( setPoint );
    
  }
  else if( rxSettings.controlMode == 1 ){
    
    setThrottle( setPoint );
    
  }
  else if( rxSettings.controlMode == 2 ){

    // Setpoint not used (PID by VESC)
    UART.nunchuck.lowerButton = cruise;
    esc.detach();

    // Make sure the motor doesn't begin to spin wrong way under high load (and don't allow cruise backwards)
    if( returnData.rpm < 0 ){

      UART.nunchuck.lowerButton = false;
      UART.nunchuck.valueY = 127;
      UART.setNunchuckValues();
      UART.setCurrent(0.0);
 
    } else{

      UART.nunchuck.valueY = map(throttle, 0, 1023, 0, 255);
      UART.setNunchuckValues();
      
    }
  }
}

void setThrottle( uint16_t throttle )
{
  if( rxSettings.controlMode == 0 ){
    
    esc.attach(throttlePin);
    esc.writeMicroseconds( map(throttle, 0, 1023, 1000, 2000) );  
  }
  else if( rxSettings.controlMode == 1 ){

    esc.attach(throttlePin);
    esc.writeMicroseconds( map(throttle, 0, 1023, 1000, 2000) ); 

  }
  else if( rxSettings.controlMode == 2 ){
    
    UART.nunchuck.valueY = map(throttle, 0, 1023, 0, 255);
    UART.nunchuck.lowerButton = false;
    esc.detach();
    UART.setNunchuckValues();

  }
}

void speedControl( uint16_t throttle , bool trigger )
{
	// Kill switch
	if( rxSettings.triggerMode == 0 ){
		if ( trigger == true || throttle < 512 ){
			setThrottle( throttle );
		}
		else{
			setThrottle( defaultThrottle );
		}
	}

	// Cruise control
	else if( rxSettings.triggerMode == 1 ){ 
    if( trigger == true ){
      
      if( cruising == false ){
        cruiseThrottle = throttle;
        cruiseRPM = returnData.rpm;
        cruising = true;
      }

      setCruise( true, cruiseThrottle, throttle );
      
    }else{
      cruising = false;
      setThrottle( throttle );
    }
	}
} 

void getUartData()
{

	if ( millis() - lastUartPull >= uartPullInterval ) {

		lastUartPull = millis();

    DEBUG_PRINT("Getting the DATA");

		// Only get what we need
		if ( UART.getVescValues() )
		{
			returnData.ampHours 		  = UART.data.ampHours;
			returnData.inpVoltage		  = UART.data.inpVoltage;
			returnData.rpm 				    = UART.data.rpm;
			returnData.tachometerAbs 	= UART.data.tachometerAbs;
		} 
		else
		{
			returnData.ampHours 		  = 0.0;
			returnData.inpVoltage     = 0.0;
			returnData.rpm 				    = 0;
			returnData.tachometerAbs  = 0;
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
	for ( int i = 0; i < numOfSettings; i++ )
	{
		setSettingValue(i, settingRules[i][0]);
	}

  rxSettings.firmVersion = VERSION;
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

  if(rxSettings.firmVersion != VERSION){
    
    setDefaultEEPROMSettings();
    
  }
	else if (rewriteSettings == true)
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
    
    default: /* Do nothing */ break;
	}
}

// Get settings value by index (usefull when iterating through settings).
int getSettingValue(uint8_t index)
{
	int value;
	switch (index) {
		case 0: value = rxSettings.triggerMode; break;
		case 1: value = rxSettings.controlMode; break;
    
    default: /* Do nothing */ break;
	}
	return value;
}

bool inRange(int val, int minimum, int maximum)
{
	return ((minimum <= val) && (val <= maximum));
}
