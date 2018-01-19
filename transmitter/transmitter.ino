#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "RF24.h"
#include "VescUart.h"

#define DEBUG

#ifdef DEBUG
	#define DEBUG_PRINT(x)  Serial.println (x)
	#include "printf.h"
#else
	#define DEBUG_PRINT(x)
#endif

// Defining the type of display used (128x32)
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const unsigned char logo[] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x80, 0x3c, 0x01,
  0xe0, 0x00, 0x07, 0x70, 0x18, 0x0e, 0x30, 0x18, 0x0c, 0x98, 0x99, 0x19,
  0x80, 0xff, 0x01, 0x04, 0xc3, 0x20, 0x0c, 0x99, 0x30, 0xec, 0xa5, 0x37,
  0xec, 0xa5, 0x37, 0x0c, 0x99, 0x30, 0x04, 0xc3, 0x20, 0x80, 0xff, 0x01,
  0x98, 0x99, 0x19, 0x30, 0x18, 0x0c, 0x70, 0x18, 0x0e, 0xe0, 0x00, 0x07,
  0x80, 0x3c, 0x01, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 
};

const unsigned char transmittingIcon[] = {
	0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x0f,
	0x33, 0x0f, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char connectedIcon[] = {
	0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x09,
	0x33, 0x09, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char noconnectionIcon[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x09,
	0x00, 0x09, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Defining struct to handle callback data (auto ack)
struct callback {
	float ampHours;
	float inpVoltage;
	long rpm;
	long tachometerAbs;
};

// Transmit and receive package
struct package {		// | Normal    | Setting 	| Dummy
	uint8_t type;		  // | 0         | 1			| 2
	uint8_t throttle;	// | Throttle  |         	| 
	uint8_t trigger;	// | Trigger   |        	| 
};

// Define package to transmit settings
struct settingPackage {
	uint8_t setting;
	uint64_t value; 
};

// Defining struct to hold stats 
struct stats {
	float maxSpeed;
	long maxRpm;
	float minVoltage;
	float maxVoltage;
};

// Defining struct to hold setting values while remote is turned on.
struct settings {
	uint8_t triggerMode;
	uint8_t batteryType;
	uint8_t batteryCells;
	uint8_t motorPoles;
	uint8_t motorPulley;
	uint8_t wheelPulley;
	uint8_t wheelDiameter;
	uint8_t controlMode;
	int minHallValue;
	int centerHallValue;
	int maxHallValue;
	uint64_t address;
};

// Defining variables for speed and distance calculation
float gearRatio;
float ratioRpmSpeed;
float ratioPulseDistance;

uint8_t currentSetting = 0;
const uint8_t numOfSettings = 13;

// Setting rules format: default, min, max.
const short settingRules[numOfSettings][3] {
	{0, 0, 1}, // 0: Killswitch or 1: cruise
	{0, 0, 1}, // 0 Li-ion & 1 LiPo
	{10, 0, 12},
	{14, 0, 250},
	{15, 0, 250},
	{40, 0, 250},
	{83, 0, 250},
	{1, 0, 2}, // 0: PPM only, 1: PPM and UART  or 2: UART only
	{0, 0, 1023},
	{512, 0, 1023},
	{1023, 0, 1023},
	{-1, 0, 0}, // No validation for pipe address (not really possible this way)
  {-1, 0, 0} // No validation for default address 
};

// Defining constants to hold the special settings, so it's easy changed thoughout the code
#define TRIGGER 0
#define MODE    7
#define ADDRESS 11
#define RESET 12

struct callback returnData;
struct package remPackage;
struct settingPackage setPackage;
struct settings txSettings;

// Pin defination
const uint8_t triggerPin = 4;
const uint8_t batteryMeasurePin = A2;
const uint8_t hallSensorPin = A3;

// Battery monitering
const float minVoltage = 3.2;
const float maxVoltage = 4.1;
const float refVoltage = 5.0;

// Defining variables for Hall Effect throttle.
short hallValue, throttle;
uint8_t hallCenterMargin = 4;
uint8_t hallMenuMargin = 100;

// Defining variables for NRF24 communication
const uint64_t defaultAddress = 0xE8E8F0F0E1LL;
const uint8_t defaultChannel = 108;
unsigned long lastTransmission;
bool connected = false;
short failCount;

// Defining variables for OLED display
String tempString;
uint8_t displayData = 0;
unsigned long lastSignalBlink;
unsigned long lastDataRotation;

// Instantiating RF24 object for NRF24 communication
RF24 radio(7, 8); // REMEMBER TO CHANGE TO 9 and 10 !!!!!!!!

// Defining variables for Settings menu
bool changeSettings = false;
bool changeSelectedSetting = false;

bool settingsLoopFlag = false;
bool settingsChangeFlag = false;
bool settingsChangeValueFlag = false;

bool signalBlink = false;


void setup() {
 
	#ifdef DEBUG
		Serial.begin(9600);
    printf_begin();
	#endif

  loadEEPROMSettings();
	
	pinMode(triggerPin, INPUT_PULLUP);
	pinMode(hallSensorPin, INPUT);
	pinMode(batteryMeasurePin, INPUT);

	u8g2.begin();
	drawStartScreen();

	if (triggerActive()) {
		changeSettings = true;
		drawTitleScreen(F("Remote Settings"));
	}

	// Start radio communication
	initiateTransmitter();
  
}

void loop() {
  
	calculateThrottlePosition();

	if (changeSettings == true) {
		// Use throttle and trigger to change settings
		controlSettingsMenu();
	}
	else
	{
    // Normal transmission. The state of the trigger is handled by the receiver. 
    remPackage.type = 0;
    remPackage.trigger = triggerActive();
    remPackage.throttle = throttle;
		
		// Transmit to receiver
		transmitToReceiver();
	}

	// Call function to update display
	updateMainDisplay();
}

// When called the throttle and trigger will be used to navigate and change settings
void controlSettingsMenu() {

	// If thumbwheel is in top position
	if (hallValue >= (txSettings.centerHallValue + hallMenuMargin) && settingsLoopFlag == false) {
		// Up
		if (changeSelectedSetting == true) {
			if(settingRules[currentSetting][0] != -1){
				short val = getSettingValue(currentSetting) + 1;
				
				if (inRange(val, settingRules[currentSetting][1], settingRules[currentSetting][2])) {
					setSettingValue(currentSetting, val);
					settingsLoopFlag = true;
				}
			}
		} else {
			if (currentSetting != 0) {
				currentSetting--;
				settingsLoopFlag = true;
			}
		}
	}

	// If thumbwheel is in bottom position
	else if (hallValue <= (txSettings.centerHallValue - hallMenuMargin) && settingsLoopFlag == false) {
		// Down
		if (changeSelectedSetting == true) {

			if(settingRules[currentSetting][0] != -1){
				short val = getSettingValue(currentSetting) - 1;
	
				if (inRange(val, settingRules[currentSetting][1], settingRules[currentSetting][2])) {
					setSettingValue(currentSetting, val);
					settingsLoopFlag = true;
				}
			}
			
		} else {
			if (currentSetting < (numOfSettings - 1)) {
				currentSetting++;
				settingsLoopFlag = true;
			}
		}
	}

	// If thumbwheel is in middle position
	else if ( inRange( hallValue, (txSettings.centerHallValue - hallMenuMargin), (txSettings.centerHallValue + hallMenuMargin) ) ) {
		settingsLoopFlag = false;
	}

	// Update selected setting to the new value 
	if ( triggerActive() ) {

    if(settingsChangeFlag == false){
      // Save settings to EEPROM
      if (changeSelectedSetting == true) {
        
        // Settings that need's to be transmitted to the recevier
        if( currentSetting == TRIGGER || currentSetting == MODE ){
          if( ! transmitSetting( currentSetting, getSettingValue(currentSetting) ) ){
            // Error! Load the old setting
            loadEEPROMSettings();
          }
        }
       
        // If new address is choosen
        else if ( currentSetting == ADDRESS ){
          // Generate new address
          uint64_t address = generateAddress();
  
          if( transmitSetting( currentSetting, address ) ){
            setSettingValue(currentSetting, address);
            initiateTransmitter();
          }else{
            // Error! Load the old address
            loadEEPROMSettings();
          }
        }
  
        // If we want to use the default address again
        else if ( currentSetting == RESET ){
          // Generate new address
          setSettingValue( ADDRESS, defaultAddress );
        }
       
        updateEEPROMSettings();
      }
  
      changeSelectedSetting = !changeSelectedSetting;
      settingsChangeFlag = true;
    }
	
	} else {
		settingsChangeFlag = false;
	}
}

void setDefaultEEPROMSettings() {
	for ( uint8_t i = 0; i < numOfSettings; i++ )
	{
		setSettingValue( i, settingRules[i][0] );
	}

	txSettings.address = defaultAddress;
	updateEEPROMSettings();
}

void loadEEPROMSettings() {
	// Load settings from EEPROM to custom struct
	EEPROM.get(0, txSettings);

	bool rewriteSettings = false;

	// Loop through all settings to check if everything is fine
	for (uint8_t i = 0; i < numOfSettings; i++) {	

		// If setting default value is -1, don't check if its valid
		if( settingRules[i][0] != -1 ){

      short val = getSettingValue(i);
    
			if (! inRange(val, settingRules[i][1], settingRules[i][2])) {
				// Setting is damaged or never written. Rewrite default.
				rewriteSettings = true;
				setSettingValue(i, settingRules[i][0] );
			}
		}
	}

	if (rewriteSettings == true) {
		updateEEPROMSettings();
	} else {
		// Calculate constants
		calculateRatios();
	}
}

// Write settings to the EEPROM then exiting settings menu.
void updateEEPROMSettings() {
	EEPROM.put(0, txSettings);
	calculateRatios();
}

// Update values used to calculate speed and distance travelled.
void calculateRatios() {
	// Gearing ratio
	gearRatio = (float)txSettings.motorPulley / (float)txSettings.wheelPulley; 
	// ERPM to Km/h
	ratioRpmSpeed = (gearRatio * 60 * (float)txSettings.wheelDiameter * 3.14156) / (((float)txSettings.motorPoles / 2) * 1000000); 
	// Pulses to km travelled
	ratioPulseDistance = (gearRatio * (float)txSettings.wheelDiameter * 3.14156) / (((float)txSettings.motorPoles * 3) * 1000000); 
}

// Get settings value by index (usefull when iterating through settings).
short getSettingValue(uint8_t index) {
	short value;
	switch (index) {
		case TRIGGER: value = txSettings.triggerMode;     break;
		case 1:       value = txSettings.batteryType;     break;
		case 2:       value = txSettings.batteryCells;    break;
		case 3:       value = txSettings.motorPoles;      break;
		case 4:       value = txSettings.motorPulley;     break;
		case 5:       value = txSettings.wheelPulley;     break;
		case 6:       value = txSettings.wheelDiameter;   break;
		case MODE:    value = txSettings.controlMode;     break;
		case 8:       value = txSettings.minHallValue;    break;
		case 9:       value = txSettings.centerHallValue; break;
		case 10:      value = txSettings.maxHallValue;    break;
	}
	return value;
}

// Set a value of a specific setting by index.
void setSettingValue(uint8_t index, uint64_t value) {
	switch (index) {
		case TRIGGER: txSettings.triggerMode = value;		  break;
		case 1:       txSettings.batteryType = value;		  break;
		case 2:       txSettings.batteryCells = value;	  break;
		case 3:       txSettings.motorPoles = value;		  break;
		case 4:       txSettings.motorPulley = value;		  break;
		case 5:       txSettings.wheelPulley = value;		  break;
		case 6:       txSettings.wheelDiameter = value;	  break;
		case MODE:    txSettings.controlMode = value;			break;
		case 8:       txSettings.minHallValue = value;	  break;
		case 9:       txSettings.centerHallValue = value;	break;
		case 10:      txSettings.maxHallValue = value;	  break;
		case ADDRESS: txSettings.address = value;		      break;
	}
}

// Check if an integer is within a min and max value
bool inRange(short val, short minimum, short maximum) {
	return ((minimum <= val) && (val <= maximum));
}

// Return true if trigger is activated, false otherwice
bool triggerActive() {
	if (digitalRead(triggerPin) == LOW)
		return true;
	else
		return false;
}

// Function used to transmit the throttle value, and receive the VESC realtime data.
void transmitToReceiver(){
	// Transmit once every 50 millisecond
	if ( millis() - lastTransmission >= 50 ) {

		lastTransmission = millis();

		bool sendSuccess = false;
		// Transmit the remPackage package
		sendSuccess = radio.write( &remPackage, sizeof(remPackage) );

		// Listen for an acknowledgement reponse (return of uart data).
		while (radio.isAckPayloadAvailable()) {
			radio.read( &returnData, sizeof(returnData) );
		}

		if (sendSuccess == true)
		{
			// Transmission was a succes
			failCount = 0;
			sendSuccess = false;

			DEBUG_PRINT( uint64ToAddress(txSettings.address) + F(": Transmission succes"));
		} else {
			// Transmission was not a succes
			failCount++;

			DEBUG_PRINT( uint64ToAddress(txSettings.address) + F(": Failed transmission"));
		}

		// If lost more than 5 transmissions, we can assume that connection is lost.
		if (failCount < 5) {
			connected = true;
		} else {
			connected = false;
		}
	}
}

bool transmitSetting(uint8_t setting, uint64_t value){
  
	uint64_t returnedValue;

	// Tell the receiver next package will be new settings
	remPackage.type = 1;

	radio.write( &remPackage, sizeof(remPackage) );
	DEBUG_PRINT( F("Transmitted setting remPackage") );
	delay(100);

	// Feed the settingPackage with the new setting
	setPackage.setting = setting;
	setPackage.value = value;

	radio.write( &setPackage, sizeof(setPackage) );
	
  if(setting == ADDRESS){
    DEBUG_PRINT( "Transmitted new setting: " + (String)setting + "=" + uint64ToAddress(value) );  
  }else{
    DEBUG_PRINT( "Transmitted new setting: " + (String)setting + "=" + uint64ToString(value) );
  }
	
	delay(100);

  // Lets clear the ack-buffer (so it can be used to confirm the new setting).
  while ( radio.isAckPayloadAvailable() ) {
    radio.read( &returnData, sizeof(returnData) );
    DEBUG_PRINT( F("Cleared buffer") );
  }
  
  delay(100);

	// Write dummy package to receive auto ack
	remPackage.type = 2; 
	radio.write( &remPackage, sizeof(remPackage) );
	DEBUG_PRINT( F("Transmitted dummy package") );
  
	delay(250);

	while ( radio.isAckPayloadAvailable() )
	{
		radio.read( &returnedValue, sizeof(returnedValue));

    if(setting == ADDRESS){
      DEBUG_PRINT( "Received auto ack: " + uint64ToAddress(returnedValue) );  
    }else{
      DEBUG_PRINT( "Received auto ack: " + uint64ToString(returnedValue) );
    }
	}

	if(returnedValue == value){
    DEBUG_PRINT( F("Setting updated on receiver") );
		return true;
	}

  DEBUG_PRINT( F("Transmitting settings failed") );
  
	return false;
}

void initiateTransmitter(){
  
  radio.begin();
  // radio.setChannel(defaultChannel);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe( txSettings.address );

  #ifdef DEBUG
    radio.printDetails();
  #endif
  
}

void updateMainDisplay()
{
	u8g2.firstPage();
 
	do {
		if ( changeSettings == true )
		{
			drawSettingsMenu();
		}
		else
		{
			drawThrottle();
			drawPage();
			drawBatteryLevel();
			drawSignal();
		}
	} while ( u8g2.nextPage() );
}

void calculateThrottlePosition()
{
	// Hall sensor reading can be noisy, lets make an average reading.
	unsigned short total = 0;
	for ( uint8_t i = 0; i < 10; i++ )
	{
		total += analogRead(hallSensorPin);
	}

	hallValue = total / 10;
	
	if ( hallValue >= txSettings.centerHallValue )
	{
		throttle = constrain( map(hallValue, txSettings.centerHallValue, txSettings.maxHallValue, 127, 255), 127, 255 );
	} else {
		throttle = constrain( map(hallValue, txSettings.minHallValue, txSettings.centerHallValue, 0, 127), 0, 127 );
	}

	// removeing center noise
	if ( abs(throttle - 127) < hallCenterMargin )
	{
		throttle = 127;
	}
}

// Function used to indicate the remotes battery level.
uint8_t batteryLevel() {
	float voltage = batteryVoltage();

	if (voltage <= minVoltage) {
		return 0;
	} else if (voltage >= maxVoltage) {
		return 100;
	} else {
		return (voltage - minVoltage) * 100 / (maxVoltage - minVoltage);
	}
}

// Function to calculate and return the remotes battery voltage.
float batteryVoltage()
{
	float batteryVoltage = 0.0;
	unsigned short total = 0;

	for (uint8_t i = 0; i < 10; i++) {
		total += analogRead(batteryMeasurePin);
	}

	batteryVoltage = (refVoltage / 1024.0) * ((float)total / 10.0);

	return batteryVoltage;
}

// To save precious SRAM we define function to get the setting titles
String getSettingTitle(uint8_t index){
	String title;
	
	switch(index){
		case 0: title = "Trigger use"; 		break;
		case 1: title = "Battery type"; 	break;
		case 2: title = "Battery cells"; 	break;
		case 3: title = "Motor poles"; 		break;
		case 4: title = "Motor pulley"; 	break;
		case 5: title = "Wheel pulley"; 	break;
		case 6: title = "Wheel diameter"; 	break;
		case 7: title = "Control mode"; 		break;
		case 8: title = "Throttle min"; 	  break;
		case 9: title = "Throttle center"; 	break;
		case 10: title = "Throttle max"; 	    break;
		case ADDRESS: title = "Generate address";  break;
    case RESET: title = "Reset address";     break;
	}

	return title;
}

void drawSettingsMenu() {
	// To save SRAM we define strings local only to be used then changing settings
	String settingValues[3][3] = {
		{"Killswitch", "Cruise", "Data toggle"},
		{"Li-ion", "LiPo", ""},
		{"PPM", "PPM and UART", "UART"},
	};
  
	String settingUnits[3] = {"S", "T", "mm"};
	String settingValue, settingUnit;

	// Used to map setting values and units to the right setting number
	uint8_t unitIdentifier[numOfSettings]  = {0,0,1,0,2,2,3,0,0,0,0,0};
	uint8_t valueIdentifier[numOfSettings] = {1,2,0,0,0,0,0,3,0,0,0,0};
	
	// Base coordinates
	uint8_t x = 0;
	uint8_t y = 10;
	
	// Local variable to store the setting value
	uint64_t value;
	
	// Print setting title
	tempString = getSettingTitle( currentSetting );
  drawString(tempString, tempString.length() + 1, x, y, u8g2_font_profont12_tr );

	// Get current setting value

  switch(currentSetting){
    case ADDRESS:
      value = txSettings.address;
      break;
  
    case RESET:
      value = defaultAddress;
      break;

    default:
      value = getSettingValue(currentSetting);
      break;
  }

	// Check if there is a text string for the setting value
	if( valueIdentifier[currentSetting] != 0 )
	{
    uint8_t index = valueIdentifier[ currentSetting ] - 1;
    settingValue = settingValues[ index ][ value ]; 
	}
	else
	{
    if(currentSetting == ADDRESS || currentSetting == RESET){
      settingValue = uint64ToAddress(value);
    }else{
		  settingValue = uint64ToString(value);
    }
	}
 
	if( unitIdentifier[ currentSetting ] != 0 ){
		settingUnit = settingUnits[ unitIdentifier[ currentSetting ] - 1 ];
	}

	// Display the setting value, and its unit
	tempString = settingValue + settingUnit;

	if ( changeSelectedSetting == true )
	{
    drawString(tempString, tempString.length() + 1, x + 10, y + 20, u8g2_font_10x20_tr );
    
    if( inRange(currentSetting, 8, 10) ){
      tempString = "(" + (String)hallValue + ")";
      drawString(tempString, 8, x + 92, y + 20, u8g2_font_profont12_tr );
    }
	}
	else
	{
		drawString(tempString, tempString.length() + 1, x, y + 20, u8g2_font_10x20_tr );
	}
}


void drawStartScreen() {
	u8g2.firstPage();
 
	do {
    
		u8g2.drawXBM( 4, 4, 24, 24, logo);

		tempString = F("Firefly remote");
    drawString(tempString, 15, 33, 21, u8g2_font_helvR10_tr );
    
	} while ( u8g2.nextPage() );
  
	delay(1500);
}

void drawTitleScreen(String title) {
	u8g2.firstPage();
 
	do {
	
    drawString(title, 20, 12, 20, u8g2_font_helvR10_tr );
    
	} while ( u8g2.nextPage() );
  
	delay(1500);
}

void drawPage() {
  
	float value;
	uint8_t decimals;
	String suffix;
	String prefix;

	short first, last;

	uint8_t x = 0;
	uint8_t y = 16;

	// Rotate the realtime data each 4s.
	if ((millis() - lastDataRotation) >= 4000) {

		lastDataRotation = millis();
		displayData++;

		if (displayData > 2) {
			displayData = 0;
		}
	}

	switch (displayData) {
		case 0:
			value = ratioRpmSpeed * returnData.rpm;
			suffix = F("KMH");
			prefix = F("SPEED");
			decimals = 1;
			break;
		case 1:
			value = ratioPulseDistance * returnData.tachometerAbs;
			suffix = F("KM");
			prefix = F("DISTANCE");
			decimals = 2;
			break;
		case 2:
			value = returnData.inpVoltage;
			suffix = F("%");
			prefix = F("BATTERY");
			decimals = 1;
			break;
	}

	// Display prefix (title)
  drawString(prefix, 10, x, y - 1, u8g2_font_profont12_tr );

	// Split up the float value: a number, b decimals.
	first = abs( floor(value) );
	last = value * pow(10, 3) - first * pow(10, 3);

	// Add leading zero
	if ( first <= 9 ) {
		tempString = "0" + (String)first;
	} else {
		tempString = (String)first;
	}

	// Display numbers
  drawString(tempString, 10, x + 55, y + 13, u8g2_font_logisoso22_tn );

	// Display decimals
  tempString = "." + (String)last;
  drawString(tempString, decimals + 2, x + 86, y - 1, u8g2_font_profont12_tr);

	// Display suffix
  drawString(suffix, 10, x + 88, y + 13, u8g2_font_profont12_tr);
}

void drawString(String text, uint8_t lenght, uint8_t x, uint8_t y, const uint8_t  *font){
  
  static char textBuffer[20];
  
  text.toCharArray(textBuffer, lenght);

  u8g2.setFont(font);
  u8g2.drawStr(x, y, textBuffer);
}

void drawThrottle() {
	uint8_t x = 0;
	uint8_t y = 18;
  uint8_t width;
  
	// Draw throttle
	u8g2.drawHLine(x, y, 52);
	u8g2.drawVLine(x, y, 10);
	u8g2.drawVLine(x + 52, y, 10);
	u8g2.drawHLine(x, y + 10, 5);
	u8g2.drawHLine(x + 52 - 4, y + 10, 5);

	if (throttle >= 127) {
		width = map(throttle, 127, 255, 0, 49);

		for (uint8_t i = 0; i < width; i++)
		{
			u8g2.drawVLine(x + i + 2, y + 2, 7);
		}
	} else {
		width = map(throttle, 0, 126, 49, 0);
   
		for (uint8_t i = 0; i < width; i++)
		{
			u8g2.drawVLine(x + 50 - i, y + 2, 7);
		}
	}
}

void drawSignal() {
	// Position on OLED
	uint8_t x = 114;
	uint8_t y = 17;

	if (connected == true) {
		if (triggerActive()) {
			u8g2.drawXBM(x, y, 12, 12, transmittingIcon);
		} else {
			u8g2.drawXBM(x, y, 12, 12, connectedIcon);
		}
	} else {
		if (millis() - lastSignalBlink > 500) {
			signalBlink = !signalBlink;
			lastSignalBlink = millis();
		}

		if (signalBlink == true) {
			u8g2.drawXBM(x, y, 12, 12, connectedIcon);
		} else {
			u8g2.drawXBM(x, y, 12, 12, noconnectionIcon);
		}
	}
}

void drawBatteryLevel() {
	// Position on OLED
	uint8_t x = 108; 
  uint8_t y = 4;

  uint8_t level = batteryLevel();

	u8g2.drawFrame(x + 2, y, 18, 9);
	u8g2.drawBox(x, y + 2, 2, 5);

	for (uint8_t i = 0; i < 5; i++) {
		uint8_t p = round((100 / 5) * i);
		if (p <= level)
		{
			u8g2.drawBox(x + 4 + (3 * i), y + 2, 2, 5);
		}
	}
}

// Generate a random address for nrf24 communication
uint64_t generateAddress()
{
  randomSeed( millis() );
  
	// Holding the address as char array
	char temp[10];

	// Char arrays with HEX digites
	const char *hexdigits = "0123456789ABCDEF";
	const char *safedigits = "12346789BCDE";

	// Generate a char array with the pipe address
	for(uint8_t i = 0 ; i < 10; i++ )
	{
		char next;
		
		// Avoid addresses that start with 0x00, 0x55, 0xAA and 0xFF.
		if(i == 0)
			next = safedigits[ random(0, 12) ];
		else if(i == 1)
			next = safedigits[ random(0, 12) ];

		// Otherwise generate random HEX digit
		else
			next = hexdigits[ random(0, 16) ];
			
		temp[i] = next;
	}
 
	// Convert hex char array to uint64_t 
	uint64_t address = StringToUint64(temp);

	return address;
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

// Convert hex String to uint64_t: http://forum.arduino.cc/index.php?topic=233813.0
uint64_t StringToUint64( char * string ){
	uint64_t x = 0;
	char c;
	
	do {
		c = hexCharToBin( *string++ );
		if (c < 0)
			break;
		x = (x << 4) | c;
	} while (1);
	
	return x;
}

char hexCharToBin(char c) {
	if (isdigit(c)) {  // 0 - 9
		return c - '0';
	} else if (isxdigit(c)) { // A-F, a-f
		return (c & 0xF) + 9;
	}
	return -1;
}

