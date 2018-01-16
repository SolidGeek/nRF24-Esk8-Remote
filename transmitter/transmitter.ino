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

static unsigned char logo_bits[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x80, 0x3c, 0x01, 0xe0, 0x00, 0x07, 0x70, 0x18, 0x0e, 0x30, 0x18, 0x0c, 0x98, 0x99, 0x19, 0x80, 0xff, 0x01, 0x04, 0xc3, 0x20, 0x0c, 0x99, 0x30, 0xec, 0xa5, 0x37, 0xec, 0xa5, 0x37, 0x0c, 0x99, 0x30, 0x04, 0xc3, 0x20, 0x80, 0xff, 0x01, 0x98, 0x99, 0x19, 0x30, 0x18, 0x0c, 0x70, 0x18, 0x0e, 0xe0, 0x00, 0x07, 0x80, 0x3c, 0x01, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static unsigned char signal_transmitting_bits[] = {
	0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x0f,
	0x33, 0x0f, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

static unsigned char signal_connected_bits[] = {
	0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x09,
	0x33, 0x09, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

static unsigned char signal_noconnection_bits[] = {
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
	uint8_t mode;
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
const uint8_t numOfSettings = 12;

// Setting rules format: default, min, max.
const short settingRules[numOfSettings][3] {
	{0, 0, 2}, // 0 Killswitch, 1 cruise & 2 data toggle
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
	{-1, 0, 0} // No validation for pipe address (not really possible this way)
};

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
const float refVoltage = 5.0; // Set to 4.5V if you are testing connected to USB, otherwise 5V (or the supply voltage)

// Defining variables for Hall Effect throttle.
short hallMeasurement, throttle;
uint8_t hallCenterMargin = 4;

// Defining variables for NRF24 communication
const uint64_t defaultAddress = 0xE8E8F0F0E1LL;
const uint8_t defaultChannel = 108;
unsigned long lastTransmission;
bool connected = false;
short failCount;

// Defining variables for OLED display
String displayString;
char displayBuffer[20];
short displayData = 0;
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

void setup() {
  
	setDefaultEEPROMSettings();
	loadEEPROMSettings();
 
	#ifdef DEBUG
		Serial.begin(9600);
    printLoadedSettings();
	#endif
	
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
	radio.begin();
	// radio.setChannel(defaultChannel);
	radio.setPALevel(RF24_PA_MAX);
	radio.enableAckPayload();
	radio.enableDynamicPayloads();
	radio.openWritingPipe(txSettings.address);

	#ifdef DEBUG
		printf_begin();
		radio.printDetails();
	#endif
}

void loop() {
  
	calculateThrottlePosition();

	if (changeSettings == true) {
		// Use throttle and trigger to change settings
		controlSettingsMenu();
	}
	else
	{
    // Normal transmission
    remPackage.type = 0;
    
		// Use throttle and trigger to drive motors
		if (triggerActive())
		{
			remPackage.throttle = throttle;
			remPackage.trigger = true;
		}
		else
		{
			// 127 is the middle position - no throttle and no brake/reverse
			remPackage.throttle = 127;
			remPackage.trigger = false;
		}
		
		// Transmit to receiver
		transmitToReceiver();
	}

	// Call function to update display
	updateMainDisplay();
}

void controlSettingsMenu() {

	// If thumbwheel is in top position
	if (hallMeasurement >= (txSettings.maxHallValue - 150) && settingsLoopFlag == false) {
		// Up
		if (changeSelectedSetting == true) {
			
			if(settingRules[currentSetting][0] != -1){
				int val = getSettingValue(currentSetting) + 1;
				
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
	else if (hallMeasurement <= (txSettings.minHallValue + 150) && settingsLoopFlag == false) {
		// Down
		if (changeSelectedSetting == true) {

			if(settingRules[currentSetting][0] != -1){
				int val = getSettingValue(currentSetting) - 1;
	
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
	else if (inRange(hallMeasurement, txSettings.centerHallValue - 100, txSettings.centerHallValue + 100)) {
		settingsLoopFlag = false;
	}


	// Update selected setting to the new value 
	if (triggerActive()) {
		if (settingsChangeFlag == false) {

			// Save settings to EEPROM
			if (changeSelectedSetting == true) {

				// Special case settings
				if( currentSetting == 0 || currentSetting == 7 ){

					if( ! transmitSetting( currentSetting, getSettingValue(currentSetting) ) ){
						// Error! Load the old setting
						loadEEPROMSettings();
					}

				}
				// If setting is to repair remote with receiver
				else if ( currentSetting == 11 )
				{
					// Generate new address
					uint64_t address = generateAddress();

					if( transmitSetting( currentSetting, address ) )
					{
						setSettingValue(currentSetting, address);
            restartReceiver();
					}
					else{
						// Error! Load the old address
						loadEEPROMSettings();
					}
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
	for ( int i = 0; i < numOfSettings; i++ )
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
	for (int i = 0; i < numOfSettings; i++) {
		int val = getSettingValue(i);

		// If setting default value is -1, don't check if its valid
		if( settingRules[i][0] != -1 ){
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

void printLoadedSettings(){
  for(int i = 0; i < numOfSettings; i++){
    if(i == 11){
      DEBUG_PRINT( uint64ToString( txSettings.address ) );
    }else{
      DEBUG_PRINT( (String)getSettingValue(i) );
    }
  }
}

// Get settings value by index (usefull when iterating through settings).
int getSettingValue(uint8_t index) {
	int value;
	switch (index) {
		case 0: value = txSettings.triggerMode;     break;
		case 1: value = txSettings.batteryType;     break;
		case 2: value = txSettings.batteryCells;    break;
		case 3: value = txSettings.motorPoles;      break;
		case 4: value = txSettings.motorPulley;     break;
		case 5: value = txSettings.wheelPulley;     break;
		case 6: value = txSettings.wheelDiameter;   break;
		case 7: value = txSettings.mode;            break;
		case 8: value = txSettings.minHallValue;    break;
		case 9: value = txSettings.centerHallValue; break;
		case 10: value = txSettings.maxHallValue;   break;
	}
	return value;
}

// Set a value of a specific setting by index.
void setSettingValue(uint8_t index, uint64_t value) {
	switch (index) {
		case 0: txSettings.triggerMode = value;		  break;
		case 1: txSettings.batteryType = value;		  break;
		case 2: txSettings.batteryCells = value;	  break;
		case 3: txSettings.motorPoles = value;		  break;
		case 4: txSettings.motorPulley = value;		  break;
		case 5: txSettings.wheelPulley = value;		  break;
		case 6: txSettings.wheelDiameter = value;	  break;
		case 7: txSettings.mode = value;			      break;
		case 8: txSettings.minHallValue = value;	  break;
		case 9: txSettings.centerHallValue = value;	break;
		case 10: txSettings.maxHallValue = value;	  break;
		case 11: txSettings.address = value;		    break;
	}
}

// Check if an integer is within a min and max value
bool inRange(int val, int minimum, int maximum) {
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

		boolean sendSuccess = false;
		// Transmit the remPackage package
		sendSuccess = radio.write(&remPackage, sizeof(remPackage));

		// Listen for an acknowledgement reponse (return of uart data).
		while (radio.isAckPayloadAvailable()) {
			radio.read(&returnData, sizeof(returnData));
		}

		if (sendSuccess == true)
		{
			// Transmission was a succes
			failCount = 0;
			sendSuccess = false;

			DEBUG_PRINT(F("Transmission succes"));
		} else {
			// Transmission was not a succes
			failCount++;

			DEBUG_PRINT(F("Failed transmission"));
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

	radio.write(&remPackage, sizeof(remPackage));
	DEBUG_PRINT(F("Transmitted setting remPackage"));
	delay(100);

	// Feed the settingPackage with the new setting
	setPackage.setting = setting;
	setPackage.value = value;

	radio.write(&setPackage, sizeof(setPackage));
	
  if(setting == 11){
    DEBUG_PRINT("Transmitted new setting: " + (String)setting + "=" + uint64ToAddress(value) );  
  }else{
    DEBUG_PRINT("Transmitted new setting: " + (String)setting + "=" + uint64ToString(value) );
  }
	
	delay(100);

  // Lets clear the ack-buffer (so it can be used to confirm the new setting).
  while (radio.isAckPayloadAvailable()) {
    radio.read(&returnData, sizeof(returnData));
    DEBUG_PRINT("Cleared buffer");
  }
  
  delay(100);

	// Write dummy package to receive auto ack
	remPackage.type = 2; 
	radio.write(&remPackage, sizeof(remPackage));
	DEBUG_PRINT(F("Transmitted dummy package"));
	delay(250);

	while (radio.isAckPayloadAvailable()) {
		radio.read(&returnedValue, sizeof(returnedValue));

    if(setting == 11){
      DEBUG_PRINT("Received auto ack: " + uint64ToAddress(returnedValue) );  
    }else{
      DEBUG_PRINT("Received auto ack: " + uint64ToString(returnedValue) );
    }
	}

	if(returnedValue == value){
    DEBUG_PRINT(F("Setting updated on receiver"));
		return true;
	}

  DEBUG_PRINT(F("Transmitting settings failed"));
	return false;
  
}

// Restart the nrf24 to transmit on new address
void restartReceiver(){
  DEBUG_PRINT("Resetting receiver address");
  radio.stopListening();
  delay(50);
  radio.openWritingPipe( txSettings.address);
}


void updateMainDisplay() {
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

void calculateThrottlePosition() {
	// Hall sensor reading can be noisy, lets make an average reading.
	int total = 0;
	for ( int i = 0; i < 10; i++ )
	{
		total += analogRead(hallSensorPin);
	}

	hallMeasurement = total / 10;
	
	if (hallMeasurement >= txSettings.centerHallValue) {
		throttle = constrain(map(hallMeasurement, txSettings.centerHallValue, txSettings.maxHallValue, 127, 255), 127, 255);
	} else {
		throttle = constrain(map(hallMeasurement, txSettings.minHallValue, txSettings.centerHallValue, 0, 127), 0, 127);
	}

	// removeing center noise
	if (abs(throttle - 127) < hallCenterMargin) {
		throttle = 127;
	}
}

// Function used to indicate the remotes battery level.
int batteryLevel() {
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
float batteryVoltage() {
	float batteryVoltage = 0.0;
	int total = 0;

	for (int i = 0; i < 10; i++) {
		total += analogRead(batteryMeasurePin);
	}

	batteryVoltage = (refVoltage / 1024.0) * ((float)total / 10.0);

	return batteryVoltage;
}

// To save precious SRAM we define function to get the setting titles
String getSettingTitle(int index){
	String title;
	
	switch(index){
		case 0: title = "Trigger use"; 		break;
		case 1: title = "Battery type"; 	break;
		case 2: title = "Battery cells"; 	break;
		case 3: title = "Motor poles"; 		break;
		case 4: title = "Motor pulley"; 	break;
		case 5: title = "Wheel pulley"; 	break;
		case 6: title = "Wheel diameter"; 	break;
		case 7: title = "Mode"; 			      break;
		case 8: title = "Throttle min"; 	  break;
		case 9: title = "Throttle center"; 	break;
		case 10: title = "Throttle max"; 	  break;
		case 11: title = "Generate address"; break;
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
	displayString = getSettingTitle( currentSetting );
	displayString.toCharArray( displayBuffer, displayString.length() + 1 );
	u8g2.setFont( u8g2_font_profont12_tr );
	u8g2.drawStr( x, y, displayBuffer );

	// Get current setting value

  if(currentSetting == 11){
    value = txSettings.address;
  }else{
    value = getSettingValue(currentSetting);
  }
	

	// Check if there is a text string for the setting value
	if( valueIdentifier[currentSetting] != 0 ){

    int index = valueIdentifier[ currentSetting ] - 1;
    settingValue = settingValues[ index ][ value ]; 
   
	}
	else
	{

    if(currentSetting == 11){
      settingValue = uint64ToAddress(value);
    }else{
		  settingValue = uint64ToString(value);
    }
	}
 
	if( unitIdentifier[ currentSetting ] != 0 ){
		settingUnit = settingUnits[ unitIdentifier[ currentSetting ] - 1 ];
	}

	// Display the setting value, and its unit
	displayString = settingValue + settingUnit;
	displayString.toCharArray( displayBuffer, displayString.length() + 1 );
	u8g2.setFont( u8g2_font_10x20_tr );

	if ( changeSelectedSetting == true )
	{
		u8g2.drawStr( x + 10, y + 20, displayBuffer );
	}
	else
	{
		u8g2.drawStr( x, y + 20, displayBuffer );
	}
}


void drawStartScreen() {
	u8g2.firstPage();
	do {
		u8g2.drawXBM( 4, 4, 24, 24, logo_bits);

		displayString = F("Esk8 remote");
		displayString.toCharArray(displayBuffer, 12);
		u8g2.setFont(u8g2_font_helvR10_tr  );
		u8g2.drawStr(34, 22, displayBuffer);
	} while ( u8g2.nextPage() );
	delay(1500);
}

void drawTitleScreen(String title) {
	u8g2.firstPage();
	do {
		title.toCharArray(displayBuffer, 20);
		u8g2.setFont(u8g2_font_helvR10_tr  );
		u8g2.drawStr(12, 20, displayBuffer);
	} while ( u8g2.nextPage() );
	delay(1500);
}

void drawPage() {
	float value;
	int decimals;
	String suffix;
	String prefix;

	int first, last;

	int x = 0;
	int y = 16;

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
			suffix = F("V");
			prefix = F("BATTERY");
			decimals = 1;
			break;
	}

	// Display prefix (title)
	displayString = prefix;
	displayString.toCharArray(displayBuffer, 10);
	u8g2.setFont(u8g2_font_profont12_tr);
	u8g2.drawStr(x, y - 1, displayBuffer);

	// Split up the float value: a number, b decimals.
	first = abs(floor(value));
	last = value * pow(10, 3) - first * pow(10, 3);

	// Add leading zero
	if (first <= 9) {
		displayString = "0" + (String)first;
	} else {
		displayString = (String)first;
	}

	// Display numbers
	displayString.toCharArray(displayBuffer, 10);
	u8g2.setFont(u8g2_font_logisoso22_tn );
	u8g2.drawStr(x + 55, y + 13, displayBuffer);

	// Display decimals
	displayString = "." + (String)last;
	displayString.toCharArray(displayBuffer, decimals + 2);
	u8g2.setFont(u8g2_font_profont12_tr);
	u8g2.drawStr(x + 86, y - 1, displayBuffer);

	// Display suffix
	displayString = suffix;
	displayString.toCharArray(displayBuffer, 10);
	u8g2.setFont(u8g2_font_profont12_tr);
	u8g2.drawStr(x + 86 + 2, y + 13, displayBuffer);
}

void drawThrottle() {
	int x = 0;
	int y = 18;

	// Draw throttle
	u8g2.drawHLine(x, y, 52);
	u8g2.drawVLine(x, y, 10);
	u8g2.drawVLine(x + 52, y, 10);
	u8g2.drawHLine(x, y + 10, 5);
	u8g2.drawHLine(x + 52 - 4, y + 10, 5);

	if (throttle >= 127) {
		int width = map(throttle, 127, 255, 0, 49);

		for (int i = 0; i < width; i++) {
			//if( (i % 2) == 0){
			u8g2.drawVLine(x + i + 2, y + 2, 7);
			//}
		}
	} else {
		int width = map(throttle, 0, 126, 49, 0);
		for (int i = 0; i < width; i++) {
			//if( (i % 2) == 0){
			u8g2.drawVLine(x + 50 - i, y + 2, 7);
			//}
		}
	}
}

bool signalBlink = false;

void drawSignal() {
	// Position on OLED
	int x = 114; int y = 17;

	if (connected == true) {
		if (triggerActive()) {
			u8g2.drawXBM(x, y, 12, 12, signal_transmitting_bits);
		} else {
			u8g2.drawXBM(x, y, 12, 12, signal_connected_bits);
		}
	} else {
		if (millis() - lastSignalBlink > 500) {
			signalBlink = !signalBlink;
			lastSignalBlink = millis();
		}

		if (signalBlink == true) {
			u8g2.drawXBM(x, y, 12, 12, signal_connected_bits);
		} else {
			u8g2.drawXBM(x, y, 12, 12, signal_noconnection_bits);
		}
	}
}

void drawBatteryLevel() {
	int level = batteryLevel();

	// Position on OLED
	int x = 108; int y = 4;

	u8g2.drawFrame(x + 2, y, 18, 9);
	u8g2.drawBox(x, y + 2, 2, 5);

	for (int i = 0; i < 5; i++) {
		int p = round((100 / 5) * i);
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
	for(int i = 0 ; i < 10; i++ )
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

