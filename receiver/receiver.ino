#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"
#include "VescUart.h"

struct vescValues {
  float ampHours;
  float inpVoltage;
  long rpm;
  long tachometerAbs;
};

RF24 radio(9, 10);
const uint64_t pipe = 0xE8E8F0F0E1LL;

bool recievedData = false;
uint32_t lastTimeReceived = 0;

int motorSpeed = 127;
int timeoutMax = 500;
int speedPin = 5;

struct bldcMeasure measuredValues;

struct vescValues data;
unsigned long lastDataCheck;

void setup() {
  SERIALIO.begin(115200);

  radio.begin();
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, pipe);
  radio.startListening();

  pinMode(speedPin, OUTPUT);
  analogWrite(speedPin, motorSpeed);
}

void loop() {

  getVescData();

  // If transmission is available
  if (radio.available())
  {
    // The next time a transmission is received on pipe, the data in gotByte will be sent back in the acknowledgement (this could later be changed to data from VESC!)
    radio.writeAckPayload(pipe, &data, sizeof(data));

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

void getVescData() {

  if (millis() - lastDataCheck >= 250) {

    lastDataCheck = millis();

    // Only transmit what we need
    if (VescUartGetValue(measuredValues)) {
      data.ampHours = measuredValues.ampHours;
      data.inpVoltage = measuredValues.inpVoltage;
      data.rpm = measuredValues.rpm;
      data.tachometerAbs = measuredValues.tachometerAbs;
    } else {
      data.ampHours = 0.0;
      data.inpVoltage = 0.0;
      data.rpm = 0;
      data.tachometerAbs = 0;
    }
  }
}
