#ifndef _REMOTE_CONSTANTS_h
#define _REMOTE_CONSTANTS_h

/** ======= Icons ======= **/

const uint8_t charging_icon[] PROGMEM = {
  0x00, 0x03, 0x00, 0x00, 0x1F, 0x00, 0x00, 0xFE, 0x00, 0x00, 0xFE, 0x0F, 
  0xFF, 0x07, 0x00, 0xF0, 0x07, 0x00, 0x80, 0x0F, 0x00, 0x00, 0x0C, 0x00, 
};

const uint8_t firefly_icon[] PROGMEM = {
  0x00, 0x00, 0x04, 0x00, 0x00, 0x04, 0x80, 0x03, 0x06, 0xF0, 0x9F, 0x03, 
  0x7C, 0xFC, 0x77, 0x0E, 0xE0, 0x1C, 0x03, 0xB0, 0x0D, 0x07, 0x30, 0x0F, 
  0x0E, 0x18, 0x06, 0x38, 0x98, 0x07, 0xF0, 0xFF, 0x0D, 0xC0, 0x7F, 0x0C, 
  0xC0, 0x18, 0x0C, 0xA0, 0x19, 0x18, 0xA0, 0x1B, 0x18, 0x60, 0x1F, 0x18, 
  0xC0, 0x1C, 0x0C, 0x80, 0x33, 0x0C, 0x00, 0x30, 0x0C, 0x00, 0x60, 0x06, 
  0x00, 0xC0, 0x06, 0x00, 0xC0, 0x03, 0x00, 0x80, 0x01
};

const uint8_t connection_icon[] PROGMEM = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x0f,
  0x33, 0x0f, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const uint8_t connecting_icon[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x09,
  0x00, 0x09, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/** ======= Settings ======= **/

const uint16_t SETTINGS_VERSION = 1;

const uint8_t SETTINGS_COUNT = 18;
const uint16_t SETTING_WAIT = 500;
const uint16_t SETTING_SCROLL_DELAY = 800;

// Setting rules format: default, min, max.
const short SETTINGS_RULES[SETTINGS_COUNT][3] {
	{0, 	0, 	2},		// Mode: 			0-2 	(0:PPM 			| 1:PPM and UART 	| 2:UART) 
	{0, 	0, 	1},		// Orientation: 	0-1 	(0:Left handed 	| 1:Right handed)
	{2, 	0, 	2},		// Top trigger: 	0-2 	(0:Disabled 	| 1:Killswitch 		| 2: Cruise)
	{1, 	0, 	2},		// Lower trigger: 	0-2 	(0:Disabled 	| 1:Killswitch 		| 2: Cruise)
	{100, 	0, 	1023},	// Throttle min: 	0-1023	(hall output)
	{512, 	0, 	1023},	// Throttle center: 0-1023  (hall output)
	{900, 	0, 	1023},	// Throttle max:	0-1023	(hall output)
	{5, 	0, 	100},	// Deadzone:		0-100	(hall value)
	{0, 	0, 	100},	// Throttle limit:	0-100	(percentage of throttle)
	{0, 	0, 	100},	// Brake limit:		0-100	(percentage of braking)
	{10, 	1, 	20},	// Battery cells:	1-20	(number of cells)
	{280, 	0, 	500},	// Min voltage:		0-500	(centivolt, divide by 100 to get voltage)
	{420, 	0, 	500},	// Max voltage:		0-500	(centivolt, divide by 100 to get voltage)
	{14, 	1, 	250},	// Motor poles:		1-250	(number of motor poles)
	{15, 	1, 	250},	// Motor pulley:	1-250	(number of pulley teeth)
	{40, 	1, 	250},	// Wheel pulley:	1-250	(number of pulley teeth)
	{90, 	1, 	1000},	// Wheel diameter:	1-1000	(diameter in millimeter)
	{2, 	0, 	60}		// Auto turnoff timer:	0-60	(0: disabled  | 1-60 minutes)
};

/** ======= Display strings ======= **/

const char MENU_TITLES[5][15] = {
	"Calibrate",
	"Pair",
	"Settings",
	"Load default",
	"Exit"
};

const char SETTING_TITLES[SETTINGS_COUNT][16] = {
	"Control mode",
	"Orientation",
	"Top trigger",
	"Lower trigger",
	"Throttle min",
	"Throttle center",
	"Throttle max",
	"Deadzone",
	"Throttle limit",
	"Brake limit",
	"Battery cells",
	"Min voltage",
	"Max voltage",
	"Motor poles",
	"Motor pulley",
	"Wheel pulley",
	"Wheel diameter",
	"Turnoff timer"
};

const char SETTING_VALUES[4][3][11] = {
  {"PPM",       "PPM & UART",   "UART only"},
  {"Left",      "Right",        ""},
	{"Disabled",  "Killswitch",   "Cruise"},
	{"Li-ion",    "Li-Po",        ""},
};

const char SETTING_UNITS[5][4] = {
	"S", "T", "mm", "V", "min"
};

/** ======= Pin definations ======= **/

const uint8_t PIN_LOWERTRIGGER 	= 4;
const uint8_t PIN_UPPERTRIGGER	= 5;
const uint8_t PIN_USBDETECT     = 6;
const uint8_t PIN_SLEEP 	      = 7;
const uint8_t PIN_VOLTAGE  	    = A2;
const uint8_t PIN_HALL 		      = A3;
const uint8_t PIN_CE 		        = 9;
const uint8_t PIN_CS 		        = 10;


/** ======= Battery monitering ======= **/

const float VOLTAGE_MIN = 3.4; // Minimum LDO input voltage
const float VOLTAGE_MAX = 4.2; // Lithium maximum charge voltage
const float VOLTAGE_REF = 3.3; // MCU supply power


/** ======= Hall sensor ======= **/

const uint16_t THROTTLE_CENTER = 512;
const uint8_t HALL_MENU_MARGIN = 100;

/** ======= NRF24 communication ======= **/

const uint64_t DEFAULT_PIPE 	= 0xE8E8F0F0E1LL;
const uint8_t DEFAULT_CHANNEL 	= 108;

class Remote;

#endif
