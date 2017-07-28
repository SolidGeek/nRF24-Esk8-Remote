#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"

U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Pin defination
const int powerLedPin = 6;
const int statusLedPin = 5;
const int triggerPin = 4;
const int batteryVoltagePin = 3;
const int batteryMeasurePin = A2;
const int hallSensorPin = A3;

unsigned int statusBlinkOff = 100;
unsigned int statusBlinkOn = 100;
bool statusBlinkState = LOW;
unsigned long lastStatusBlink = 0;
const float refVolt = 5.0;
float batteryVoltage = 0.0;

// Defining variables for Hall Effect throttle.
int hallCenter, hallTop, hallBottom, hallMeasurement, hallSpeed, hallOffset;
int hallCenterMargin = 4;

// Defining variables for NRF24 communication
bool sendSuccess = false;
bool returnSuccess = false;
bool connected = false;
byte gotByte = 0;
int failedCounter;
const uint64_t pipe = 0xE8E8F0F0E1LL;

// Instantiating RF24 object for NRF24 communication
RF24 radio(9, 10);

void setup() {
  Serial.begin(9600);
  u8g2.begin();

  pinMode(statusLedPin, OUTPUT);
  pinMode(triggerPin, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT);
  pinMode(powerLedPin, OUTPUT);
  pinMode(batteryVoltagePin, OUTPUT);
  pinMode(batteryMeasurePin, INPUT);
  digitalWrite(batteryVoltagePin, LOW);

  // Turn on the powerLED.
  digitalWrite(powerLedPin, HIGH);

  // Start radio communication at MAX POWER!!!
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe(pipe);
  
  // Find median of Hall Sensor in middle position
  calibrateHallSensor();

  // Set the statusLED to blink
  statusBlink(1);
}

char lcdBuffer[20];
String lcdString;

void loop() {

  if(triggerActive())
  {
    getHallSpeed();
  }
  else
  {
    hallSpeed = 127;  
  }

  // Call function to check remote battery voltage
  checkBatteryVoltage();
  Serial.println(batteryVoltage);

  // Call function to update display and LED
  updateDisplay();

  // Transmit to receiver
  transmitToVesc();
}

void updateDisplay(){

  if (connected == true) {
    statusBlink(2);
  } else {
    statusBlink(1);
  }
  
  lcdString = "V: " + (String)batteryVoltage;
  lcdString.toCharArray(lcdBuffer, 20);

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_gb24st_t_1);
    u8g2.drawStr(10, 20, lcdBuffer);
  } while ( u8g2.nextPage() );

}

boolean triggerActive(){
  if(digitalRead(triggerPin) == LOW)
    return true;
  else
    return false;
}

void checkBatteryVoltage(){
  digitalWrite(batteryVoltagePin, HIGH);
  int total = 0;
  
  for(int i = 0; i < 10; i++){
      total += analogRead(batteryMeasurePin);
      delay(150);
      Serial.println(analogRead(batteryMeasurePin));
  }

  batteryVoltage = (5.02 / 1024.0) * ((float)total / 10.0);

  delay(2000);
  
  digitalWrite(batteryVoltagePin, LOW);
}

void calibrateHallSensor(){
  int total = 0;
  
  for(int i = 0; i < 10; i++){
      total += analogRead(hallSensorPin);
      delay(50);
  }

  hallCenter = total / 10;
  hallTop = hallCenter + 300;
  hallBottom = hallCenter - 300;
}

void transmitToVesc() {

  sendSuccess = radio.write(&hallSpeed, sizeof(hallSpeed));

  while (radio.isAckPayloadAvailable())
  {
    radio.read(&gotByte, sizeof(gotByte));
    returnSuccess = true;
  }

  if (sendSuccess == true)
  {
    failedCounter = 0;
    // Serial.println("Send successfully! " + (String)gotByte);
    sendSuccess = false;
  } else {
    failedCounter++;
  }

  if (failedCounter == 0) {
    connected = true;
  } else {
    connected = false;
  }
}

void getHallSpeed() {
  hallMeasurement = analogRead(hallSensorPin);

  if (hallMeasurement > hallTop) {
    hallTop = hallMeasurement;
  }
  if (hallMeasurement < hallBottom) {
    hallBottom = hallMeasurement;
  }

  hallOffset = hallCenter - ((hallTop + hallBottom) / 2);


  hallSpeed = map((hallMeasurement - hallOffset), hallBottom, hallTop, 0, 255);
  // put your main code here, to run repeatedly:

  if (abs(hallSpeed - 127) < hallCenterMargin) {
    hallSpeed = 127;
  }
  if (hallSpeed < 0) {
    hallSpeed = 0;
  }
  if (hallSpeed > 255) {
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

    if(statusBlinkOff == 0){
    statusBlinkState = HIGH;
  }
  digitalWrite(statusLedPin, statusBlinkState);
}

