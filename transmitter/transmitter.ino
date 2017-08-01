#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include "RF24.h"

U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

static unsigned char u8g_logo_bits[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x80, 0x3c, 0x01, 0xe0, 0x00, 0x07, 0x70, 0x18, 0x0e, 0x30, 0x18, 0x0c, 0x98, 0x99, 0x19, 0x80, 0xff, 0x01, 0x04, 0xc3, 0x20, 0x0c, 0x99, 0x30, 0xec, 0xa5, 0x37, 0xec, 0xa5, 0x37, 0x0c, 0x99, 0x30, 0x04, 0xc3, 0x20, 0x80, 0xff, 0x01, 0x98, 0x99, 0x19, 0x30, 0x18, 0x0c, 0x70, 0x18, 0x0e, 0xe0, 0x00, 0x07, 0x80, 0x3c, 0x01, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

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
const float minVoltage = 2.8;
const float maxVoltage = 4.2;
const float refVoltage = 5.0; // Set to 4.5V if you are testing connected to USB, otherwise 5V (or the supply voltage)

// Defining variables for Hall Effect throttle.
int hallCenter, hallTop, hallBottom, hallMeasurement, hallSpeed, hallOffset;
int hallCenterMargin = 4;

// Defining variables for NRF24 communication
bool connected = false;
byte gotByte = 0;
int failCount;
const uint64_t pipe = 0xE8E8F0F0E1LL;
unsigned long lastTransmission;

// Defining variables for OLED display
char displayBuffer[20];
String displayString;

// Instantiating RF24 object for NRF24 communication
RF24 radio(7, 8);

void setup() {
  pinMode(statusLedPin, OUTPUT);
  pinMode(triggerPin, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT);
  pinMode(powerLedPin, OUTPUT);
  pinMode(batteryMeasurePin, INPUT);
  // Turn on the powerLED.
  digitalWrite(powerLedPin, HIGH);

  u8g2.begin();

  drawStartScreen();
  
  // Start radio communication at MAX POWER!!!
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe(pipe);
  radio.setRetries(15, 15);

  // Find median of Hall Sensor in middle position
  calibrateHallSensor();

  // Set the statusLED to blink
  statusBlink(1);
}


void loop() {
  if (triggerActive())
  {
    calculateHallSpeed();
  }
  else
  {
    // 127 is the middle position - no throttle and no brake/reverse
    hallSpeed = 127;
  }

  // Call function to update display and LED
  updateDisplay();

  // Transmit to receiver
  transmitToVesc();
}

void updateDisplay() {
  if (connected == true) {
    statusBlink(2);
  } else {
    statusBlink(1);
  }

  u8g2.firstPage();
  do {
    drawSpeed();
    drawBatteryLevel();
    drawSignalStrenght();
    
  } while ( u8g2.nextPage() );
}

void drawStartScreen(){
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

void drawSpeed(){
  // Position on OLED
  int x = 3; int y = 26;
  
  displayString = (String)hallSpeed;
  displayString.toCharArray(displayBuffer, 8);

  u8g2.setFont(u8g2_font_fur20_tn);
  u8g2.drawStr(x, y, displayBuffer);    
}

void drawSignalStrenght(){
  // Position on OLED
  int x = 60; int y = 16;

  if(connected == true){
    displayString = "Connected";
  }else{
    displayString = "No signal";
  }

  displayString.toCharArray(displayBuffer, 12);

  u8g2.setFont(u8g2_font_7x13_mr);
  u8g2.drawStr(x, y+12, displayBuffer);
}

void drawBatteryLevel() {
  int level = batteryLevel();

  // Position on OLED
  int x = 70; int y = 2;

  u8g2.drawFrame(x + 2, y, 21, 10);
  u8g2.drawBox(x, y + 2, 2, 6);

  for(int i = 0; i < 6; i++){
    int p = round((100/6) * i);
    if(p <= level)
    {
       u8g2.drawBox(x + 4 + (3*i), y + 2, 2, 6); 
    }
  }

  displayString = (String)level + "%";
  displayString.toCharArray(displayBuffer, 8);

  u8g2.setFont(u8g2_font_7x13_mr);
  u8g2.drawStr(x + 27, y + 10, displayBuffer);
}

boolean triggerActive() {
  if (digitalRead(triggerPin) == LOW)
    return true;
  else
    return false;
}

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

float batteryVoltage() {
  float batteryVoltage = 0.0;
  int total = 0;

  for (int i = 0; i < 10; i++) {
    total += analogRead(batteryMeasurePin);
  }

  batteryVoltage = (refVoltage / 1024.0) * ((float)total / 10.0);

  return batteryVoltage;
}

void calibrateHallSensor() {
  int total = 0;

  for (int i = 0; i < 10; i++) {
    total += analogRead(hallSensorPin);
    delay(50);
  }

  hallCenter = total / 10;
  hallTop = hallCenter + 300;
  hallBottom = hallCenter - 300;
}

void transmitToVesc() {
  gotByte = 0;

  if(millis() - lastTransmission >= 50){

    lastTransmission = millis();
    
    boolean sendSuccess = false;
    sendSuccess = radio.write(&hallSpeed, sizeof(hallSpeed));
  
    while (radio.isAckPayloadAvailable())
    {
      radio.read(&gotByte, sizeof(gotByte));
    }
  
    if (sendSuccess == true)
    {
      failCount = 0;
      sendSuccess = false;
      Serial.println("Forbundet");
    } else {
      failCount++;
       Serial.println("Fejlet: " + (String)failCount);
    }
  
    if (failCount < 5) {
      connected = true;
      
    } else {
      connected = false;
     
    }
  }
}

void calculateHallSpeed() {

  // Hall sensor reading can be noisy, lets make an average reading.
  int total = 0;
  for(int i = 0; i < 10; i++){
    total += analogRead(hallSensorPin);
  }
  hallMeasurement = total / 10;

  // Update hallTop and hallBottom if higher values are read.
  if (hallMeasurement > hallTop) {
    hallTop = hallMeasurement;
  }
  if (hallMeasurement < hallBottom) {
    hallBottom = hallMeasurement;
  }

  // Calculate offset needed to achieve 127 at the no-throttle position.
  hallOffset = hallCenter - ((hallTop + hallBottom) / 2);

  // Calculate speed (0-255)
  hallSpeed = map((hallMeasurement - hallOffset), hallBottom, hallTop, 0, 255);

  // Making sure the speed is within 0 and 255, and removeing center noise by adding a margin
  if (abs(hallSpeed - 127) < hallCenterMargin) {
    hallSpeed = 127;
  }
  else if (hallSpeed < 0) {
    hallSpeed = 0;
  }
  else if (hallSpeed > 255) {
    hallSpeed = 255;
  }
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
