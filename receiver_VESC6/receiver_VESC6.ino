#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"
#include "VescUart.h"
#include <Servo.h>
Servo myservo;

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
const uint64_t pipe = 0xA8A8FEFEE1LL; // Please change this to your own pipe address
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
  float avgMotorCurrent;
};

struct bldcMeasure uartData;
struct callback returnData;

bool recievedData = false;
uint32_t lastTimeReceived = 0;

int motorSpeed = 1500 ; // This is in microseconds
int timeoutMax = 500;
int throttlePin = 5;
//int resetTrigger = 0;

struct bldcMeasure measuredValues;

struct vescValues data;
unsigned long lastDataCheck;

void setup() {

  SetSerialPort(&SERIALIO);
  SERIALIO.begin(115200);

  myservo.attach(throttlePin);
  myservo.write(motorSpeed);
  //pinMode(throttlePin, OUTPUT); // This is the old analogWrite method
  //analogWrite(throttlePin, motorSpeed); // This is the old analogWrite method  

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

    // Write the PWM signal to the ESC (0-255) // This is the old analogWrite method
    //analogWrite(throttlePin, motorSpeed); // This is the old analogWrite method
    // Maps motorspeed signal from remote from 0-255 to 1000-2000ms
    motorSpeed = map(motorSpeed, 0, 255, 1000, 2000);
    // Writes microseconds to throttle pin
    myservo.writeMicroseconds(motorSpeed);

  }
  else if ((millis() - lastTimeReceived) > timeoutMax)
  {
    // No speed is received within the timeout limit.
    //analogWrite(throttlePin, motorSpeed);
    myservo.writeMicroseconds(1500);
  }
}

void initiateReceiver() {

  // Start radio communication
  radio.begin();
  // radio.setChannel(defaultChannel);
  radio.setDataRate(RF24_250KBPS);
  //radio.setChannel(108);
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
      data.ampHours         = 0.0;
      data.inpVoltage       = 0.0;
      data.rpm              = 0;
      data.tachometerAbs    = 0;
      data.avgMotorCurrent  = 0;
    }
  }
}
