#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"

U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

RF24 radio(7, 8);
const uint64_t pipe = 0xE8E8F0F0E1LL;

int motorSpeed = 0;
int statusLedPin = 5;
int killSwitchPin = 3;
int hallSensorPin = A7;

unsigned int statusBlinkOff = 100;
unsigned int statusBlinkOn = 100;
bool statusBlinkState = LOW;
unsigned long lastStatusBlink = 0;

int hallCenter = 575;
int hallTop = 950;
int hallBottom = 150;
int hallCenterMargin = 5;
int hallMeasurement, hallSpeed, hallOffset;


bool sendSuccess = false;
bool returnSuccess = false;
bool connected = false;
int failedCounter;
byte gotByte;


void setup() {
  // Serial.begin(9600);
  u8g2.begin();

  pinMode(statusLedPin, OUTPUT);
  pinMode(killSwitchPin, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT);


  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe(pipe);

  // radio.printDetails();

  statusBlink(1); // Connecting
}

char lcdBuffer[20];
String lcdString;

void loop() {
  
  if(digitalRead(killSwitchPin) == LOW){
    getHallSpeed();
  }else{
    hallSpeed = 127;  
  }
  
  lcdString = "Speed: " + (String)hallSpeed;
  lcdString.toCharArray(lcdBuffer, 20);

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(10, 24, lcdBuffer);
  } while ( u8g2.nextPage() );

  
  transmitToVesc();
 
  
  if (connected == true) {
    statusBlink(2);
  } else {
    statusBlink(1);
  }

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
