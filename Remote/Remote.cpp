#include "Remote.h"

Remote::Remote(void)
{
  Display.init( this );
}

void Remote::begin(void)
{
  Display.begin();
  Settings.load();
  
  pinMode(PIN_USBDETECT, INPUT);
}

uint8_t Remote::batteryPercentage(void)
{
	measureVoltage();

	if(voltage >= VOLTAGE_MAX)
		return 100;
	else
		return (uint8_t)( (voltage - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN) * 100.0);
}

void Remote::calculateThrottle()
{
  measureHallOutput();

  if ( hallOutput >= Settings.throttleCenter )
  {
    throttle = constrain( map(hallOutput, Settings.throttleCenter, Settings.throttleMax, THROTTLE_CENTER, 1023), THROTTLE_CENTER, 1023 );
  } else {

    throttle = constrain( map(hallOutput, Settings.throttleMin, Settings.throttleCenter, 0, THROTTLE_CENTER), 0, THROTTLE_CENTER );
  }

  // Remove hall center noise
  if ( abs(throttle - THROTTLE_CENTER) < HALL_NOISE_MARGIN )
  {
 
    throttle = THROTTLE_CENTER;
  }
  
}

uint16_t Remote::getThrottle()
{
  this->calculateThrottle();
  
  return this->throttle;
}

void Remote::measureHallOutput(void){

  uint16_t sum = 0;
  uint8_t samples = 5;
  
  for ( uint8_t i = 0; i < samples; i++ )
  {
    sum += analogRead(PIN_HALL);
  }
  
  uint16_t mean = sum / samples;
  hallRaw = mean;
  
  // Smooths the hallValue with a filter
  hallOutput = EMA(mean, hallOutput, 0.75); 

}

void Remote::measureVoltage(void){

	uint32_t sum;
  uint8_t samples = 5;

	for (int i = 0; i < samples; ++i){
		sum += analogRead(PIN_VOLTAGE);
	}

	float mean = sum/samples;

	// Multiplying by 2 because of the voltage divider
	voltage = mean / 1023.0 * VOLTAGE_REF * 2;

}


bool Remote::usbConnected(){

  if( digitalRead(PIN_USBDETECT) )
    return true;
  else
    return false;
  
}

/* Exponential moving weighted average */
int Remote::EMA( int newSample, int oldSample, float alpha ){
  return (int)((alpha * (float)newSample) + (1.0-alpha) * (float)oldSample);  
}
