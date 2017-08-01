#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"

// 9 = CE-pin - 10 = CSN-pin
RF24 radio(9, 10);
const uint64_t pipe = 0xE8E8F0F0E1LL;

bool recievedData = false;
uint32_t lastTimeReceived = 0;

int motorSpeed = 127;
int timeoutMax = 500;
int speedPin = 5;
byte gotByte = 1;

void setup() {
  radio.begin();
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, pipe);
  radio.setRetries(15, 15);
  radio.startListening();

  pinMode(speedPin, OUTPUT);
  analogWrite(speedPin, motorSpeed);
}

void loop() {
  // If transmission is available
  if (radio.available())
  {
    // The next time a transmission is received on pipe, the data in gotByte will be sent back in the acknowledgement (this could later be changed to data from VESC!)
    radio.writeAckPayload(pipe, &gotByte, sizeof(gotByte));

    // Read the actual message
    radio.read(&motorSpeed, sizeof(motorSpeed));
    recievedData = true;
  }

  if (recievedData == true)
  {
    // A speed is received from the transmitter (remote).
    
    lastTimeReceived = millis();
    recievedData = false;

    // Write the PWM signal to the ESC (0-255).
    analogWrite(speedPin, motorSpeed);
  }
  else if ((millis() - lastTimeReceived) > timeoutMax)
  {
    // No speed is received within the timeout limit.
    motorSpeed = 127;
    analogWrite(speedPin, motorSpeed);
  }
}
