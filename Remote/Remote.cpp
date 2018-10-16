#include "Remote.h"

Remote::Remote(void){

  display.initiate(this);
  // settings.initiate(this);
}

void Remote::begin(void){
  display.begin();
}

uint8_t Remote::batteryPercentage(void){

	measureVoltage();

	if(voltage >= VOLTAGE_MAX)
		return 100;
	else
		return (uint8_t)( (voltage - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN) * 100.0);

}

void Remote::throttlePosition(){

  measureHallOutput();
  
  Serial.println(hallValue);
}

void Remote::measureHallOutput(void){

  uint16_t sum = 0;
  uint8_t samples = 5;
  
  for ( uint8_t i = 0; i < samples; i++ )
  {
    sum += analogRead(PIN_HALL);
  }
  
  uint16_t mean = sum / samples;

  Serial.print(mean);
  Serial.print(',');

  // Smooths the hallValue with a filter
  hallValue = EMA(mean, hallValue, 0.35);

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


/* Exponential moving weighted average */
int Remote::EMA( int newSample, int oldSample, float alpha ){
  return (int)((alpha * (float)newSample) + (1.0-alpha) * (float)oldSample);  
}
