#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"
#include "VescUart.h"

//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.println (x)
#include "printf.h"
#else
#define DEBUG_PRINT(x)
#endif

#define SERIALIO Serial

struct vescValues {
  float ampHours;
  float inpVoltage;
  long rpm;
  long tachometerAbs;
  float avgMotorCurrent;
};

RF24 radio(9, 10);
//int radioChannel = 108; // Above most WiFi frequencies
const uint64_t pipe = 0xA8A8F0F0E1LL;
const uint8_t defaultChannel = 108;
uint32_t timeoutTimer = 0;

// Last time data was pulled from VESC
unsigned long lastUartPull;

// Defining struct to handle callback data (auto ack)
struct callback {
  float ampHours;
  float inpVoltage;
  long rpm;
  long tachometerAbs;
};

struct bldcMeasure uartData;
struct callback returnData;

bool recievedData = false;
uint32_t lastTimeReceived = 0;

int motorSpeed = 127;
int timeoutMax = 500;
int throttlePin = 5;
//int resetTrigger = 0;

struct bldcMeasure measuredValues;

struct vescValues data;
unsigned long lastDataCheck;

void setup() {

  SetSerialPort(&SERIALIO);
  SERIALIO.begin(115200);

  pinMode(throttlePin, OUTPUT);
  analogWrite(throttlePin, motorSpeed);

  initiateReceiver();
}

void loop() {

  getVescData();

  // If transmission is available
  while (radio.available())
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

    //radio.writeAckPayload(1, &returnData, sizeof(returnData));

    // Write the PWM signal to the ESC (0-255).
    analogWrite(throttlePin, motorSpeed);
  }
  else if ((millis() - lastTimeReceived) > timeoutMax)
  {
    // No speed is received within the timeout limit.
    motorSpeed = 127;
    analogWrite(throttlePin, motorSpeed);
  }
}


void initiateReceiver() {

  radio.begin();
  // radio.setChannel(defaultChannel);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, pipe);
  radio.startListening();

}

void getVescData() {

  if (millis() - lastDataCheck >= 250) {

    lastDataCheck = millis();

    // Only transmit what we need
    if ( VescUartGetValue(uartData) )
    {
      data.ampHours         = uartData.ampHours;
      data.inpVoltage       = uartData.inpVoltage;
      data.rpm              = uartData.rpm;
      data.tachometerAbs    = uartData.tachometerAbs;
      data.avgMotorCurrent  = uartData.avgMotorCurrent;
    }
    else
    {
      data.ampHours       = 0.0;
      data.inpVoltage     = 0.0;
      data.rpm            = 0;
      data.tachometerAbs  = 0;
    }
  }
}
