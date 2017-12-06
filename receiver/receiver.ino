#include <SPI.h>
#include <EEPROM.h>
#include <nRF24L01.h>
#include "RF24.h"
#include "VescUart.h"

// Defining struct to hold UART data.
struct uart {
  float ampHours;
  float inpVoltage;
  long rpm;
  long tachometerAbs;
};

struct package {
  byte throttle; // Throttle value
  bool trigger; // Trigger state

  bool newSettings; // Update receiver settings
  byte mode; // PPM only, PPM and UART  or UART only
  byte triggerMode; // Killswitch, Cruise or data toggle
  uint64_t address; // New pipe
};

RF24 radio(9, 10);
const uint64_t defaultPipe = 0xE8E8F0F0E1LL;

bool recievedData = false;
uint32_t lastTimeReceived = 0;

const int defaultThrottle = 127;
const byte defaultMode = 1;
const short timeoutMax = 500;
const byte throttlePin = 5;
const byte pairingPin = 6;

struct bldcMeasure measuredValues;
struct uart uartData;
struct package remoteData;
struct settings receiverSettings;

unsigned long lastDataCheck;

void setup() {
  // Use this to rewrite default receiver settings
  setDefaultEEPROMSettings(); 

  // VESC uart communication
  // SERIALIO.begin(115200);

  Serial.begin(9600);

  radio.begin();
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, receiverSettings.address);
  radio.startListening();

  pinMode(throttlePin, OUTPUT);
  pinMode(pairingPin, INPUT_PULLUP);

  // Set default throttle in startup
  analogWrite(throttlePin, defaultThrottle);
}

void loop() {
  // If transmission is available
  if (radio.available())
  {
    // Read the actual message
    radio.read(&remoteData, sizeof(remoteData));
    recievedData = true;
  }

  if(recievedData == true){
    printPackage();
    
    // Update settings 
    if(remoteData.newSettings == true){
      if(remoteData.mode != receiverSettings.mode){
      }
      if(remoteData.address != receiverSettings.address){
      }
    }else{
      
      switch(receiverSettings.mode){
        // PPM only
        case 0:
          setThrottlePPM();
          break;
        // PPM and UART
        case 1:
          setThrottlePPM();
          getUartData();         
          break;
        // UART only
        case 2:
          setThrottleUART();
          getUartData();
          break;
      }

      // The next time a transmission is received on pipe, the data in uartData will be sent back in the acknowledgement 
      radio.writeAckPayload(receiverSettings.address, &uartData, sizeof(uartData));
    }

    // A speed is received from the transmitter (remote).

    lastTimeReceived = millis();
    recievedData = false;
    
  }
}

void setThrottlePPM(){
  if (recievedData == true)
  {
    // Write the PWM signal to the ESC (0-255).
    analogWrite(throttlePin, remoteData.throttle);
  }
  else if ((millis() - lastTimeReceived) > timeoutMax)
  {
    // No speed is received within the timeout limit.
    remoteData.throttle = 127;
    analogWrite(throttlePin, remoteData.throttle);
  }
}

void setThrottleUART(){
  
}


void getUartData() {
  if (millis() - lastDataCheck >= 250) {

    lastDataCheck = millis();

    // Only transmit what we need
    if (VescUartGetValue(measuredValues)) {
      uartData.ampHours = measuredValues.ampHours;
      uartData.inpVoltage = measuredValues.inpVoltage;
      uartData.rpm = measuredValues.rpm;
      uartData.tachometerAbs = measuredValues.tachometerAbs;
    } else {
      uartData.ampHours = 0.0;
      uartData.inpVoltage = 0.0;
      uartData.rpm = 0;
      uartData.tachometerAbs = 0;
    }
  }
}

void printPackage(){
  Serial.println();
  Serial.println(remoteData.throttle);
  Serial.println();
  Serial.println(remoteData.newSettings);
  Serial.println(remoteData.mode);
  Serial.println(uint64ToString(remoteData.address));
}


String uint64ToString(uint64_t number)
{
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  return String(part1, HEX) + String(part2, HEX);
}

// Settings functions

void setDefaultEEPROMSettings() {
  receiverSettings.mode = defaultMode;
  receiverSettings.address = defaultPipe;

  updateEEPROMSettings();
}

void loadEEPROMSettings() {
  bool rewriteSettings = false;
  
  // Load settings from EEPROM to custom struct
  EEPROM.get(0, receiverSettings);

  if (! inRange(receiverSettings.mode, 0, 2) ){
    // Setting is damaged or never written. Rewrite default.
    rewriteSettings = true;
    receiverSettings.mode = defaultMode;
  }

  if (rewriteSettings == true) {
    updateEEPROMSettings();
  }
}

// Write settings to the EEPROM then exiting settings menu.
void updateEEPROMSettings() {
  EEPROM.put(0, receiverSettings);
}

bool inRange(int val, int minimum, int maximum) {
  return ((minimum <= val) && (val <= maximum));
}

