#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "RF24.h"

// #define DEBUG

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
} returnData;

// Transmit and receive package
struct package {		// | Normal 	| Setting 	| Dummy
	uint8_t type;		// | 0 			| 1			| 2
	uint16_t throttle;	// | Throttle 	| 			| 
	uint8_t trigger;	// | Trigger 	| 			| 
} remPackage;

// Define package to transmit settings
struct settingPackage {
	uint8_t setting;
	uint64_t value; 
} setPackage;

// Defining struct to hold stats 
struct stats {
	float maxSpeed;
	long maxRpm;
	float minVoltage;
	float maxVoltage;
};

// Defining struct to hold setting values while remote is turned on.
struct settings {
	uint8_t triggerMode; 	// 0
	uint8_t batteryType; 	// 1
	uint8_t batteryCells; 	// 2
	uint8_t motorPoles;		// 3
	uint8_t motorPulley;	// 4
	uint8_t wheelPulley; 	// 5
	uint8_t wheelDiameter;	// 6
	uint8_t controlMode; 	// 7
	short minHallValue; 		// 8
	short centerHallValue;	// 9
	short maxHallValue; 		// 10
	uint64_t address; 		// 11
} txSettings;

// Defining constants to hold the special settings, so it's easy changed thoughout the code
#define TRIGGER 0
#define MODE    7
#define ADDRESS 11
#define RESET 12

// Defining variables to hold values for speed and distance calculation
float gearRatio;
float ratioRpmSpeed;
float ratioPulseDistance;

uint8_t currentSetting = 0;
const uint8_t numOfSettings = 13;

// Setting rules format: default, min, max.
const short rules[numOfSettings][3] {
	{0, 0, 1}, 		// 0: Killswitch 	| 1: Cruise control
	{0, 0, 1}, 		// 0: Li-ion 		| 1: LiPo
	{10, 0, 12},	// Cell count
	{10, 0, 12},	// Motor poles
	{14, 0, 250},	// Motor pully
	{40, 0, 250},	// Wheel pulley
	{83, 0, 250},	// Wheel diameter
	{1, 0, 1}, 		// 0: PPM only or 1: PPM and UART
	{200, 0, 1023},	// Min hall value
	{500, 0, 1023},	// Center hall value
	{800, 0, 1023},	// Max hall value
	{-1, 0, 0}, 	  // Address
	{-1, 0, 0} 	    // Set default address
};

const char titles[numOfSettings][17] = {
  "Trigger use", "Battery type", "Battery cells", "Motor poles", "Motor pulley",
  "Wheel pulley", "Wheel diameter", "Control mode", "Throttle min", "Throttle center",
  "Throttle max", "Generate address", "Reset address"
};

const uint8_t unitIdentifier[numOfSettings]  = {0,0,1,0,2,2,3,0,0,0,0,0,0};
const uint8_t valueIdentifier[numOfSettings] = {1,2,0,0,0,0,0,3,0,0,0,0,0};

const char stringValues[3][2][13] = {
  {"Killswitch", "Cruise"},
  {"Li-ion", "LiPo"},
  {"PPM", "PPM and UART"},
};

const char settingUnits[3][3] = {"S", "T", "mm"};
const char dataSuffix[3][4] = {"KMH", "KM", "%"};
const char dataPrefix[3][9] = {"SPEED", "DISTANCE", "BATTERY"};

// Pin defination
const uint8_t triggerPin = 4;
const uint8_t batteryMeasurePin = A2;
const uint8_t hallSensorPin = A3;
const uint8_t CE = 9;
const uint8_t CS = 10;

// Battery monitering
const float minVoltage = 3.2;
const float maxVoltage = 4.1;
const float refVoltage = 5.0;

// Defining variables for Hall Effect throttle.
uint16_t hallValue, throttle;
uint8_t hallCenterMargin = 16;
uint8_t hallMenuMargin = 100;

// Defining variables for NRF24 communication
const uint64_t defaultAddress = 0xE8E8F0F0E1LL;
const uint8_t defaultChannel = 108;
unsigned long lastTransmission;
bool connected = false;
short failCount;

// Defining variables for OLED display
String tString;
uint8_t displayData = 0;
unsigned long lastSignalBlink;
unsigned long lastDataRotation;

// Instantiating RF24 object for NRF24 communication
RF24 radio(CE, CS);

// Defining variables for Settings menu
bool changeSettings = false;
bool changeThisSetting = false;
bool settingsLoopFlag = false;
bool settingsChangeFlag = false;
bool settingsChangeValueFlag = false;
unsigned short settingWaitDelay = 500;

bool signalBlink = false;

// Used to set position of graphics
uint8_t x, y;

void setup() {

  // setDefaultEEPROMSettings();
 
	#ifdef DEBUG
		Serial.begin(9600);
    while (!Serial){};
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
		drawTitleScreen("Settings");
	}

	// Start radio communication
	initiateTransmitter();

  delay(500);
}

void loop() {

	calculateThrottlePosition();

	if (changeSettings == true) {
		// Use throttle and trigger to change settings
		controlSettingsMenu();
	}
	else
	{
		// Normal transmission. The state of the trigger, cruise and throttle is handled by the receiver. 
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
		if (changeThisSetting == true) {
			if(rules[currentSetting][0] != -1){
				short val = getSettingValue(currentSetting) + 1;
				
				if (inRange(val, rules[currentSetting][1], rules[currentSetting][2])) {
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
		if (changeThisSetting == true) {

			if(rules[currentSetting][0] != -1){
				short val = getSettingValue(currentSetting) - 1;
	
				if (inRange(val, rules[currentSetting][1], rules[currentSetting][2])) {
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
	if ( triggerActive() )
	{
		if(settingsChangeFlag == false)
		{
			// Save settings to EEPROM
			if (changeThisSetting == true)
			{
				// Settings that needs to be transmitted to the recevier
				if( currentSetting == TRIGGER || currentSetting == MODE ){
					if( ! transmitSetting( currentSetting, getSettingValue(currentSetting) ) ){
						// Error! Load the old setting
						loadEEPROMSettings();
					}
				}

				// If new address is choosen
				else if ( currentSetting == ADDRESS )
				{
					// Generate new address
					uint64_t address = generateAddress();

					if( transmitSetting( currentSetting, address ) )
					{
						setSettingValue(currentSetting, address);
						initiateTransmitter();
					}
					else
					{
						// Error! Load the old address
						loadEEPROMSettings();
					}
				}

				// If we want to use the default address again
				else if ( currentSetting == RESET )
				{
					// Generate new address
					setSettingValue( ADDRESS, defaultAddress );
				}

				updateEEPROMSettings();
			}
		  
			changeThisSetting = !changeThisSetting;
			settingsChangeFlag = true;
	    }
	} 
	else 
	{
		settingsChangeFlag = false;
	}
}

void setDefaultEEPROMSettings() {
	for ( uint8_t i = 0; i < numOfSettings; i++ )
	{
		setSettingValue( i, rules[i][0] );
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
		if( rules[i][0] != -1 ){

			short val = getSettingValue(i);
		
			if (! inRange(val, rules[i][1], rules[i][2])) {
				// Setting is damaged or never written. Rewrite default.
				rewriteSettings = true;
				setSettingValue(i, rules[i][0] );
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
		case TRIGGER: 	value = txSettings.triggerMode; break;
		case 1: 		value = txSettings.batteryType; 	  break;
		case 2: 		value = txSettings.batteryCells; 	  break;
		case 3: 		value = txSettings.motorPoles; 		  break;
		case 4: 		value = txSettings.motorPulley; 	  break;
		case 5: 		value = txSettings.wheelPulley; 	  break;
		case 6: 		value = txSettings.wheelDiameter; 	break;
		case MODE:  value = txSettings.controlMode; 	  break;
		case 8: 		value = txSettings.minHallValue; 	  break;
		case 9: 		value = txSettings.centerHallValue; break;
		case 10: 		value = txSettings.maxHallValue; 	  break;
	}
	return value;
}

// Set a value of a specific setting by index.
void setSettingValue(uint8_t index, uint64_t value) {
	switch (index) {
		case TRIGGER: 	txSettings.triggerMode = value; 	  break;
		case 1: 		    txSettings.batteryType = value; 	  break;
		case 2: 		    txSettings.batteryCells = value; 	  break;
		case 3: 		    txSettings.motorPoles = value; 		  break;
		case 4: 		    txSettings.motorPulley = value; 	  break;
		case 5: 		    txSettings.wheelPulley = value; 	  break;
		case 6: 		    txSettings.wheelDiameter = value;	  break;
		case MODE: 		  txSettings.controlMode = value; 	  break;
		case 8: 		    txSettings.minHallValue = value; 	  break;
		case 9: 		    txSettings.centerHallValue = value; break;
		case 10: 		    txSettings.maxHallValue = value; 	  break;
		case ADDRESS: 	txSettings.address = value; 		    break;

    default: /* Do nothing */ break;
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

// Function used to transmit the remPackage and receive auto acknowledgement.
void transmitToReceiver(){
	// Transmit once every 50 millisecond
	if ( millis() - lastTransmission >= 50 ) {

		lastTransmission = millis();

		// Transmit the remPackage
		if ( radio.write( &remPackage, sizeof(remPackage) ) )
		{

			// Listen for an acknowledgement reponse (return of uart data).
			while (radio.isAckPayloadAvailable()) {
				radio.read( &returnData, sizeof(returnData) );
			}

			// Transmission was a succes
			failCount = 0;

			DEBUG_PRINT( uint64ToAddress(txSettings.address) + ": Transmission succes");
		} else {
			// Transmission was not a succes
			failCount++;

			DEBUG_PRINT( uint64ToAddress(txSettings.address) +  + ": Failed transmission");
     
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
	unsigned long beginTime = millis(); 
	bool payloadSend = false;
	bool ackRecieved = false;
 
	// Lets clear the ack-buffer (so it can be used to confirm the new setting).
	while ( radio.isAckPayloadAvailable() && settingWaitDelay >= ( millis() - beginTime) ) {
		radio.read( &returnData, sizeof(returnData) );
		delay(100);
	}

	// Feed the setPackage with the new setting
	setPackage.setting = setting;
	setPackage.value = value;

	// Tell the receiver next package will be new settings
	remPackage.type = 1;

	beginTime = millis(); 
  
	while ( !payloadSend && settingWaitDelay >= (millis() - beginTime) ){
		if( radio.write( &remPackage, sizeof(remPackage)) ){
			payloadSend = true;
		}
	}

	// ** Begin transmitting new setting **

	if(payloadSend == true){

		DEBUG_PRINT( F("TX --> New setting") );

		// Transmit setPackage to receiver
		beginTime = millis(); 

		while ( !ackRecieved && settingWaitDelay >= ( millis() - beginTime) ){
			// Write setPackage until an acknowledgement is received (or timeout is reached)
			if( radio.write( &setPackage, sizeof(setPackage) ) ){

				delay(100);

				while( radio.isAckPayloadAvailable() && !ackRecieved ){
					DEBUG_PRINT( F("TX <-- Acknowledgement") );
					radio.read( &setPackage, sizeof(setPackage) );
					ackRecieved = true;
				}
			}
		}
	}


	// Check if the receiver Acknowledgement data is matching
	if( ackRecieved && setPackage.setting == setting && setPackage.value == value ){

    DEBUG_PRINT( F("Setting confirmed") );
		payloadSend = false;

    // Wait a little
    delay(500);

		// Send confirmation to the receiver
		beginTime = millis(); 

		while ( !payloadSend && settingWaitDelay >= (millis() - beginTime) ){

			remPackage.type = 2;

			if(radio.write( &remPackage, sizeof(remPackage))){
				payloadSend = true;
				DEBUG_PRINT( F("TX --> Confirmation") );
			}

			delay(100);
		}

		if( payloadSend == true) {
			// Success
			DEBUG_PRINT( F("Setting done") );
			return true;
		}
	}

	return false;

}

void initiateTransmitter(){

	radio.begin();
	radio.setChannel(defaultChannel);
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
		throttle = constrain( map(hallValue, txSettings.centerHallValue, txSettings.maxHallValue, 512, 1023), 512, 1023 );
	} else {
		throttle = constrain( map(hallValue, txSettings.minHallValue, txSettings.centerHallValue, 0, 512), 0, 512 );
	}

  DEBUG_PRINT(String(throttle));

	// removeing center noise
	if ( abs(throttle - 512) < hallCenterMargin )
	{
		throttle = 512;
	}
}

// Function used to indicate the remotes battery level.
uint8_t batteryLevel() {

  unsigned short total = 0;

  for (uint8_t i = 0; i < 10; i++) {
    total += analogRead(batteryMeasurePin);
  }

	float voltage = (refVoltage / 1024.0) * ((float)total / 10.0);

	if (voltage <= minVoltage) {
		return 0;
	} else if (voltage >= maxVoltage) {
		return 100;
	} else {
		return (voltage - minVoltage) * 100 / (maxVoltage - minVoltage);
	}
}

float batteryPackPercentage( float voltage ){

	float maxCellVoltage = 4.2;
	float minCellVoltage;

	if(txSettings.batteryType == 0){
		// Li-ion
		minCellVoltage = 2.8; 
	}
	else
	{
		// Li-po
		minCellVoltage = 3.4;
	}

	float percentage = (100 - ( (maxCellVoltage * txSettings.batteryCells - voltage)/((maxCellVoltage - minCellVoltage)*10) ) * 100);

	if(percentage > 100.0){
		return 100.0;  
	}else if (percentage < 0.0){
		return 0.0;
	}else{
		return percentage;
	}
}

void drawSettingsMenu() {
	// Local variables to store the setting value and unit
	uint64_t value;

	x = 0;
	y = 10;
	
	// Print setting title
	u8g2.setFont(u8g2_font_profont12_tr);
	u8g2.drawStr(x, y, titles[ currentSetting ] );

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
		tString = stringValues[ index ][ value ]; 
	}
	else
	{
		if(currentSetting == ADDRESS || currentSetting == RESET){
			tString = uint64ToAddress(value);
		}else{
			tString = uint64ToString(value);
		}
	}

	if( unitIdentifier[ currentSetting ] != 0 ){
		tString += settingUnits[ unitIdentifier[ currentSetting ] - 1 ];
	}

	if ( changeThisSetting == true )
	{
		drawString(tString, tString.length(), x + 10, y + 20, u8g2_font_10x20_tr );

    // If setting has something to do with the hallValue
		if( inRange(currentSetting, 8, 10) ){
			tString = "(" + String(hallValue) + ")";
			drawString(tString, tString.length(), x + 92, y + 20, u8g2_font_profont12_tr );
		}
	}
	else
	{
		drawString(tString, tString.length(), x, y + 20, u8g2_font_10x20_tr );
	}
}

void drawStartScreen() {
	u8g2.firstPage();
 
	do {
    
		u8g2.drawXBM( 4, 4, 24, 24, logo);

    u8g2.setFont(u8g2_font_10x20_tr);
    u8g2.drawStr(35, 21, "Firefly");

	} while ( u8g2.nextPage() );

  delay(1500);
}

void drawTitleScreen(String title) {
	u8g2.firstPage();
 
	do {

		drawString(title, 20, 12, 20, u8g2_font_10x20_tr );

	} while ( u8g2.nextPage() );

	delay(1500);
}


void drawPage() {
	uint8_t decimals;
	float value;

	short first, last;

	x = 0;
	y = 16;

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
			decimals = 1;
		break;
		case 1:
			value = ratioPulseDistance * returnData.tachometerAbs;
			decimals = 2;
		break;
		case 2:
			value = batteryPackPercentage( returnData.inpVoltage );
			decimals = 1;
		break;
	}

	// Display prefix (title)
	u8g2.setFont(u8g2_font_profont12_tr);
	u8g2.drawStr(x, y-1, dataPrefix[ displayData ] );

	// Split up the float value: a number, b decimals.
	first = abs( floor(value) );
	last = value * pow(10, 3) - first * pow(10, 3);

	// Add leading zero
	if ( first <= 9 ) {
		tString = "0" + String(first);
	} else {
		tString = first;
	}

	// Display numbers
	drawString(tString, 10, x + 55, y + 13, u8g2_font_logisoso22_tn );

	// Display decimals
	tString = ".";
	tString += last;
	drawString(tString, decimals + 2, x + 86, y - 1, u8g2_font_profont12_tr);

	// Display suffix
	u8g2.setFont(u8g2_font_profont12_tr);
	u8g2.drawStr(x + 88, y + 13, dataSuffix[ displayData ] );
}

void drawString(String string, uint8_t lenght, uint8_t x, uint8_t y, const uint8_t *font){

	static char cache[20];

	string.toCharArray(cache, lenght + 1);

	u8g2.setFont(font);
	u8g2.drawStr(x, y, cache);

}

void drawThrottle() {
	
	x = 0;
	y = 18;

	uint8_t width;
  
	// Draw throttle
	u8g2.drawHLine(x, y, 52);
	u8g2.drawVLine(x, y, 10);
	u8g2.drawVLine(x + 52, y, 10);
	u8g2.drawHLine(x, y + 10, 5);
	u8g2.drawHLine(x + 52 - 4, y + 10, 5);

	if (throttle >= 512) {
		width = map(throttle, 512, 1023, 0, 49);

		for (uint8_t i = 0; i < width; i++)
		{
			u8g2.drawVLine(x + i + 2, y + 2, 7);
		}
	} else {
		width = map(throttle, 0, 511, 49, 0);
   
		for (uint8_t i = 0; i < width; i++)
		{
			u8g2.drawVLine(x + 50 - i, y + 2, 7);
		}
	}
}

void drawSignal() {
	// Position on OLED
	x = 114;
	y = 17;

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
	x = 108; 
	y = 4;

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
	return StringToUint64(temp);
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

