#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"

RF24 radio(7, 8);
const uint64_t pipe = 0xE8E8F0F0E1LL;

bool recievedData = false;
uint32_t lastTimeReceived = 0;

int motorSpeed = 127;
int timeoutMax = 500;
int speedPin = 5;
byte gotByte = 1;

void setup() {
  Serial.begin(9600);

  radio.begin();
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, pipe);

  radio.startListening();

  pinMode(speedPin, OUTPUT);
  analogWrite(speedPin, motorSpeed);
}

void loop() {

  radio.writeAckPayload(pipe, &gotByte, sizeof(gotByte));

  while (radio.available())
  {
    radio.read(&motorSpeed, sizeof(motorSpeed));
    recievedData = true;
  }

  if (recievedData == true)
  {
    lastTimeReceived = millis();
    recievedData = false;

    analogWrite(speedPin, motorSpeed);

    // Serial.println(motorSpeed);
  }
  else if ((millis() - lastTimeReceived) > timeoutMax)
  {
    motorSpeed = 127;
    analogWrite(speedPin, motorSpeed);
    
    // Serial.println("Remote signal lost");
  }

}
