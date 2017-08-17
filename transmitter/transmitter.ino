#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "RF24.h"
#include "VescUart.h"

U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

static unsigned char u8g_logo_bits[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x80, 0x3c, 0x01, 0xe0, 0x00, 0x07, 0x70, 0x18, 0x0e, 0x30, 0x18, 0x0c, 0x98, 0x99, 0x19, 0x80, 0xff, 0x01, 0x04, 0xc3, 0x20, 0x0c, 0x99, 0x30, 0xec, 0xa5, 0x37, 0xec, 0xa5, 0x37, 0x0c, 0x99, 0x30, 0x04, 0xc3, 0x20, 0x80, 0xff, 0x01, 0x98, 0x99, 0x19, 0x30, 0x18, 0x0c, 0x70, 0x18, 0x0e, 0xe0, 0x00, 0x07, 0x80, 0x3c, 0x01, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static unsigned char wifi_bits[] = {
  0xc0, 0xff, 0x00, 0xf0, 0xff, 0x03, 0x7c, 0x80, 0x0f, 0x1e, 0x00, 0x1e,
  0x07, 0x00, 0x38, 0x03, 0x3f, 0x30, 0xc0, 0xff, 0x00, 0xf0, 0xe1, 0x03,
  0x78, 0x80, 0x07, 0x18, 0x00, 0x06, 0x00, 0x1e, 0x00, 0x80, 0x7f, 0x00,
  0xc0, 0xe1, 0x00, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00,
  0x00, 0x12, 0x00, 0x00, 0x12, 0x00, 0x00, 0x0c, 0x00
};

static unsigned char wifi_nosignal_bits[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00,
  0x00, 0x12, 0x00, 0x00, 0x12, 0x00, 0x00, 0x0c, 0x00
};

static unsigned char wifi_transmitting_bits[] = {
  0xc0, 0xff, 0x00, 0xf0, 0xff, 0x03, 0x7c, 0x80, 0x0f, 0x1e, 0x00, 0x1e,
  0x07, 0x00, 0x38, 0x03, 0x3f, 0x30, 0xc0, 0xff, 0x00, 0xf0, 0xe1, 0x03,
  0x78, 0x80, 0x07, 0x18, 0x00, 0x06, 0x00, 0x1e, 0x00, 0x80, 0x7f, 0x00,
  0xc0, 0xe1, 0x00, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00,
  0x00, 0x1e, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x0c, 0x00
};

struct vescValues {
  float ampHours;
  float inpVoltage;
  long rpm;
  long tachometerAbs;
};

struct settings {
  byte triggerMode;
  byte batteryType;
  byte batteryCells;
  byte motorPoles;
  byte motorPulley;
  byte wheelPulley;
  byte wheelDiameter;
  bool useUart;
  int minHallValue;
  int centerHallValue;
  int maxHallValue;
};

const int numOfSettings = 11;

float gearRatio;
float ratioRpmSpeed;
float ratioPulseDistance;

int currentSetting = 0;

String settingPages[numOfSettings][2] = {
  {"Trigger",         ""},
  {"Battery type",    ""},
  {"Battery cells",   "S"},
  {"Motor poles",     ""},
  {"Motor pulley",    "T"},
  {"Wheel pulley",    "T"},
  {"Wheel diameter",  "mm"},
  {"UART data",       ""},
  {"Throttle min",    ""},
  {"Throttle center", ""},
  {"Throttle max",    ""}
};

// Setting rules format: default, min, max.
int settingRules[numOfSettings][3] {
  {0, 0, 3}, // Killswitch, cruise & data toggle
  {0, 0, 1}, // Li-ion & LiPo
  {10, 0, 12},
  {14, 0, 250},
  {15, 0, 250},
  {40, 0, 250},
  {83, 0, 250},
  {1, 0, 1}, // Yes or no
  {0, 0, 1023},
  {512, 0, 1023},
  {1023, 0, 1023}
};

struct vescValues data;
struct settings remoteSettings;

// Pin defination
const int powerLedPin = 6;
const int statusLedPin = 5;
const int triggerPin = 4;
const int batteryMeasurePin = A2;
const int hallSensorPin = A3;

unsigned int statusBlinkOff = 100;
unsigned int statusBlinkOn = 100;
bool statusBlinkState = LOW;
unsigned long lastStatusBlink = 0;

// Battery monitering
const float minVoltage = 3.2;
const float maxVoltage = 4.1;
const float refVoltage = 5.0; // Set to 4.5V if you are testing connected to USB, otherwise 5V (or the supply voltage)

// Defining variables for Hall Effect throttle.
int hallMeasurement, throttle;
int hallCenterMargin = 4;

// Defining variables for NRF24 communication
bool connected = false;
int failCount;
int failed = 0;
const uint64_t pipe = 0xE8E8F0F0E1LL;
unsigned long lastTransmission;

// Defining variables for OLED display
char displayBuffer[20];
String displayString;

// Instantiating RF24 object for NRF24 communication
RF24 radio(7, 8);

// Mode
bool changeSettings = false;
bool changeSelectedSetting = false;

bool settingsLoopFlag = false;
bool settingsChangeFlag = false;
bool settingsChangeValueFlag = false;

void setup() {
  // setDefaultEEPROMSettings();

  loadEEPROMSettings();

  pinMode(statusLedPin, OUTPUT);
  pinMode(triggerPin, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT);
  pinMode(powerLedPin, OUTPUT);
  pinMode(batteryMeasurePin, INPUT);
  // Turn on the powerLED.
  digitalWrite(powerLedPin, HIGH);

  u8g2.begin();

  drawStartScreen();

  if (triggerActive()) {
    changeSettings = true;
    drawTitleScreen("Remote Settings");
  }

  // Start radio communication at MAX POWER!!!
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe(pipe);

  // Set the statusLED to blink
  statusBlink(1);
}

void loop() {

  calculateThrottlePosition();

  if (changeSettings == true) {
    // Use throttle and trigger to change settings
    controlSettingsMenu();
  }
  else
  {
    // Use throttle and trigger to drive motors
    if (triggerActive())
    {
      throttle = throttle;
    }
    else
    {
      // 127 is the middle position - no throttle and no brake/reverse
      throttle = 127;
    }
    // Transmit to receiver
    transmitToVesc();
  }

  // Call function to update display and LED
  updateMainDisplay();
}

void controlSettingsMenu() {
  if (triggerActive()) {
    if (settingsChangeFlag == false) {

      // Save settings to EEPROM
      if (changeSelectedSetting == true) {
        updateEEPROMSettings();
      }

      changeSelectedSetting = !changeSelectedSetting;
      settingsChangeFlag = true;
    }
  } else {
    settingsChangeFlag = false;
  }

  if (hallMeasurement >= (remoteSettings.maxHallValue - 50) && settingsLoopFlag == false) {
    // Up
    if (changeSelectedSetting == true) {
      int val = getSettingValue(currentSetting) + 1;

      if (inRange(val, settingRules[currentSetting][1], settingRules[currentSetting][2])) {
        setSettingValue(currentSetting, val);
        settingsLoopFlag = true;
      }
    } else {
      if (currentSetting != 0) {
        currentSetting--;
        settingsLoopFlag = true;
      }
    }
  }
  else if (hallMeasurement <= (remoteSettings.minHallValue + 50) && settingsLoopFlag == false) {
    // Down
    if (changeSelectedSetting == true) {
      int val = getSettingValue(currentSetting) - 1;

      if (inRange(val, settingRules[currentSetting][1], settingRules[currentSetting][2])) {
        setSettingValue(currentSetting, val);
        settingsLoopFlag = true;
      }
    } else {
      if (currentSetting < (numOfSettings - 1)) {
        currentSetting++;
        settingsLoopFlag = true;
      }
    }
  } else if (inRange(hallMeasurement, remoteSettings.centerHallValue - 50, remoteSettings.centerHallValue + 50)) {
    settingsLoopFlag = false;
  }
}

void drawSettingNumber() {
  // Position on OLED
  int x = 2; int y = 10;

  // Draw current setting number box
  u8g2.drawRFrame(x + 102, y - 10, 22, 32, 4);

  // Draw current setting number
  displayString = (String)(currentSetting + 1);
  displayString.toCharArray(displayBuffer, displayString.length() + 1);

  u8g2.setFont(u8g2_font_10x20_tf);
  u8g2.drawStr(x + 108, 22, displayBuffer);
}

void drawSettingsMenu() {
  // Position on OLED
  int x = 0; int y = 10;

  // Draw setting title
  displayString = settingPages[currentSetting][0];
  displayString.toCharArray(displayBuffer, displayString.length() + 1);

  u8g2.setFont(u8g2_font_7x13_tf);
  u8g2.drawStr(x, y, displayBuffer);

  int val = getSettingValue(currentSetting);

  displayString = (String)val + "" + settingPages[currentSetting][1];
  displayString.toCharArray(displayBuffer, displayString.length() + 1);
  u8g2.setFont(u8g2_font_10x20_tf );

  if (changeSelectedSetting == true) {
    u8g2.drawStr(x + 10, y + 20, displayBuffer);
  } else {
    u8g2.drawStr(x, y + 20, displayBuffer);
  }
}

void setDefaultEEPROMSettings() {
  for (int i = 0; i < numOfSettings; i++) {
    setSettingValue(i, settingRules[i][0]);
  }

  updateEEPROMSettings();
}

void loadEEPROMSettings() {
  // Load settings from EEPROM to custom struct
  EEPROM.get(0, remoteSettings);

  bool rewriteSettings = false;

  // Loop through all settings to check if everything is fine
  for (int i = 0; i < numOfSettings; i++) {
    int val = getSettingValue(i);

    if (! inRange(val, settingRules[i][1], settingRules[i][2])) {
      // Setting is damaged or never written. Rewrite default.
      rewriteSettings = true;
      setSettingValue(i, settingRules[i][0]);
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
  EEPROM.put(0, remoteSettings);
  calculateRatios();
}

// Update values used to calculate speed and distance travelled.
void calculateRatios() {
  gearRatio = (float)remoteSettings.motorPulley / (float)remoteSettings.wheelPulley;

  ratioRpmSpeed = (gearRatio * 60 * (float)remoteSettings.wheelDiameter * 3.14156) / (((float)remoteSettings.motorPoles / 2) * 1000000); // ERPM to Km/h

  ratioPulseDistance = (gearRatio * (float)remoteSettings.wheelDiameter * 3.14156) / (((float)remoteSettings.motorPoles * 3) * 1000000); // Pulses to km travelled
}

// Get settings value by index (usefull when iterating through settings).
int getSettingValue(int index) {
  int value;
  switch (index) {
    case 0: value = remoteSettings.triggerMode;     break;
    case 1: value = remoteSettings.batteryType;     break;
    case 2: value = remoteSettings.batteryCells;    break;
    case 3: value = remoteSettings.motorPoles;      break;
    case 4: value = remoteSettings.motorPulley;     break;
    case 5: value = remoteSettings.wheelPulley;     break;
    case 6: value = remoteSettings.wheelDiameter;   break;
    case 7: value = remoteSettings.useUart;         break;
    case 8: value = remoteSettings.minHallValue;    break;
    case 9: value = remoteSettings.centerHallValue; break;
    case 10: value = remoteSettings.maxHallValue;   break;
  }
  return value;
}

// Set a value of a specific setting by index.
void setSettingValue(int index, int value) {
  switch (index) {
    case 0: remoteSettings.triggerMode = value;     break;
    case 1: remoteSettings.batteryType = value;     break;
    case 2: remoteSettings.batteryCells = value;    break;
    case 3: remoteSettings.motorPoles = value;      break;
    case 4: remoteSettings.motorPulley = value;     break;
    case 5: remoteSettings.wheelPulley = value;     break;
    case 6: remoteSettings.wheelDiameter = value;   break;
    case 7: remoteSettings.useUart = value;         break;
    case 8: remoteSettings.minHallValue = value;    break;
    case 9: remoteSettings.centerHallValue = value; break;
    case 10: remoteSettings.maxHallValue = value;   break;
  }
}

// Check if an integer is within a min and max value
bool inRange(int val, int minimum, int maximum) {
  return ((minimum <= val) && (val <= maximum));
}

// Return true if trigger is activated, false otherwice
boolean triggerActive() {
  if (digitalRead(triggerPin) == LOW)
    return true;
  else
    return false;
}

// Function used to transmit the throttle value, and receive the VESC realtime data.
void transmitToVesc() {
  // Transmit once every 50 millisecond
  if (millis() - lastTransmission >= 50) {

    lastTransmission = millis();

    boolean sendSuccess = false;
    // Transmit the speed value (0-255).
    sendSuccess = radio.write(&throttle, sizeof(throttle));

    // Listen for an acknowledgement reponse (return of VESC data).
    while (radio.isAckPayloadAvailable()) {
      radio.read(&data, sizeof(data));
    }

    if (sendSuccess == true)
    {
      // Transmission was a succes
      failCount = 0;
      sendSuccess = false;
    } else {
      // Transmission was not a succes
      failCount++;
      failed++;
    }

    // If lost more than 5 transmissions, connection is lost.
    if (failCount < 5) {
      connected = true;
    } else {
      connected = false;
    }
  }
}

void calculateThrottlePosition() {
  // Hall sensor reading can be noisy, lets make an average reading.
  int total = 0;
  for (int i = 0; i < 10; i++) {
    total += analogRead(hallSensorPin);
  }
  hallMeasurement = total / 10;

  if (hallMeasurement >= remoteSettings.centerHallValue) {
    throttle = constrain(map(hallMeasurement, remoteSettings.centerHallValue, remoteSettings.maxHallValue, 127, 255), 127, 255);
  } else {
    throttle = constrain(map(hallMeasurement, remoteSettings.minHallValue, remoteSettings.centerHallValue, 0, 127), 0, 127);
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

void updateMainDisplay() {
  if (connected == true) {
    statusBlink(2);
  } else {
    statusBlink(1);
  }

  u8g2.firstPage();
  do {

    if (changeSettings == true) {
      drawSettingsMenu();
      drawSettingNumber();
    } else {
      drawSpeed();
      drawBatteryLevel();
      drawSignalStrenght();
    }

  } while ( u8g2.nextPage() );
}

void drawStartScreen() {
  u8g2.firstPage();
  do {
    u8g2.drawXBM( 4, 4, 24, 24, u8g_logo_bits);

    displayString = "Esk8 remote";
    displayString.toCharArray(displayBuffer, 12);
    u8g2.setFont(u8g2_font_helvR10_tf  );
    u8g2.drawStr(34, 22, displayBuffer);
  } while ( u8g2.nextPage() );
  delay(1500);
}

void drawTitleScreen(String title) {
  u8g2.firstPage();
  do {
    title.toCharArray(displayBuffer, 20);
    u8g2.setFont(u8g2_font_helvR10_tf  );
    u8g2.drawStr(12, 20, displayBuffer);
  } while ( u8g2.nextPage() );
  delay(1500);
}

int displayData = 0;
unsigned long lastDataRotation;

void drawSpeed() {
  // Rotate the realtime data each 2s.
  if ((millis() - lastDataRotation) >= 2000) {

    lastDataRotation = millis();
    displayData++;

    if (displayData > 2) {
      displayData = 0;
    }
  }

  int decimals;
  float value;
  String suffix;

  switch (displayData) {
    case 0:
      value = ratioRpmSpeed * data.rpm;
      suffix = "km/h";
      decimals = 1;
      break;
    case 1:
      value = ratioPulseDistance * data.tachometerAbs;
      suffix = "km";
      decimals = 2;
      break;
    case 2:
      value = data.inpVoltage;
      suffix = "V";
      decimals = 1;
      break;
  }

  // Position on OLED
  int x = 3; int y = 16;

  displayString = String(value, decimals) + " " + suffix;
  displayString.toCharArray(displayBuffer, 10);

  u8g2.setFont(u8g2_font_helvR10_tf);
  u8g2.drawStr(x, y, displayBuffer);

  u8g2.drawHLine(x, y + 5, 64);

  if (throttle >= 127) {
    int width = map(throttle, 127, 255, 1, 64);
    u8g2.drawBox(x, y + 5, width, 4);
  } else {
    int width = map(throttle, 0, 126, 64, 1);
    u8g2.drawBox(x + 64 - width, y + 5, width, 4);
  }
}

void drawSignalStrenght() {
  // Position on OLED
  int x = 72; int y = 6;

  if (triggerActive()) {

    if (connected == true) {
      u8g2.drawXBM(x, y, 22, 19, wifi_transmitting_bits);
    } else {
      u8g2.drawXBM(x, y, 22, 19, wifi_nosignal_bits);
    }
  } else {
    if (connected == true) {
      u8g2.drawXBM(x, y, 22, 19, wifi_bits);
    } else {
      u8g2.drawXBM(x, y, 22, 19, wifi_nosignal_bits);
    }
  }
}

void drawBatteryLevel() {
  int level = batteryLevel();

  // Position on OLED
  int x = 100; int y = 3;

  u8g2.drawFrame(x + 2, y, 21, 10);
  u8g2.drawBox(x, y + 2, 2, 6);

  for (int i = 0; i < 6; i++) {
    int p = round((100 / 6) * i);
    if (p <= level)
    {
      u8g2.drawBox(x + 4 + (3 * i), y + 2, 2, 6);
    }
  }
  displayString = (String)level + "%";
  displayString.toCharArray(displayBuffer, 8);

  u8g2.setFont(u8g2_font_7x13_tf);
  u8g2.drawStr(x, y + 25, displayBuffer);
}

void statusBlink(int state) {
  switch (state) {
    case 0:
      statusBlinkOff = 100;
      statusBlinkOn = 100;
      break;
    case 1:
      statusBlinkOff = 100;
      statusBlinkOn = 500;
      break;
    case 2:
      statusBlinkOff = 0;
      statusBlinkOn = 1000;
      break;
  }

  if ((lastStatusBlink + statusBlinkOff) < millis() && statusBlinkState == LOW) {
    statusBlinkState = HIGH;
    lastStatusBlink = millis();
  }
  if ((lastStatusBlink + statusBlinkOn) < millis() && statusBlinkState == HIGH) {
    statusBlinkState = LOW;
    lastStatusBlink = millis();
  }
  if (statusBlinkOff == 0) {
    statusBlinkState = HIGH;
  }

  digitalWrite(statusLedPin, statusBlinkState);
}
