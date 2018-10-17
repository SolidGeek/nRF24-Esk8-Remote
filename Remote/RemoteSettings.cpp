#include "Remote.h"
#include "RemoteSettings.h"

RemoteSettings::RemoteSettings( void ){}

void RemoteSettings::load()
{
  // Load settings from EEPROM to this
  EEPROM.get(0, *this);

  if ( versionNum != SETTINGS_VERSION ){
    this->reset();
  }
  else
  {
    // Setting version is okay, now check setting integrity
    bool rewriteSettings = false;

    // Loop through all settings to check if everything is fine
    for (uint8_t i = 0; i < SETTINGS_COUNT; i++)
    {
      uint16_t value = this->getValue(i);

      if (!this->inRange(value, i))
      {
        // Setting is damaged or never written. Rewrite default.
        rewriteSettings = true;
        this->setValue(i, SETTINGS_RULES[i][0]);
      }
    }

    if (rewriteSettings == true){
      this->save();
    }
  }
}

void RemoteSettings::save()
{
  this->versionNum = SETTINGS_VERSION;
  EEPROM.put(0, *this);
}

void RemoteSettings::reset()
{
  for (uint8_t i = 0; i < SETTINGS_COUNT; i++)
  {
    this->setValue(i, SETTINGS_RULES[i][0]);
  }
   
  this->save();
}

void RemoteSettings::printSettings()
{
  for(uint8_t i = 0; i < SETTINGS_COUNT; i++)
  {
    Serial.print('\t'); Serial.print(i); Serial.print(':');
    Serial.println( getValue(i) );  
  }
  
  Serial.print("Version: "); Serial.println( this->versionNum );
}

void RemoteSettings::setValue( uint8_t index, uint16_t value )
{
  uint16_t val = constrain( value, SETTINGS_RULES[index][1], SETTINGS_RULES[index][2] );

  switch(index){
    case 0: this->mode          = val; break;
    case 1: this->displayRotate = val; break;
    case 2: this->upperTrigger  = val; break;
    case 3: this->lowerTrigger  = val; break;
    case 4: this->throttleMin  = val; break;
    case 5: this->throttleCenter  = val; break;
    case 6: this->throttleMax  = val; break;
    case 7: this->throttleDeadzone  = val; break;
    case 8: this->throttleLimit  = val; break;
    case 9: this->brakeLimit  = val; break;
    case 10: this->batteryCells  = val; break;
    case 11: this->batteryMinVoltage  = val; break;
    case 12: this->batteryMaxVoltage  = val; break;
    case 13: this->motorPoles  = val; break;
    case 14: this->pulleyMotor  = val; break;
    case 15: this->pulleyWheel  = val; break;
    case 16: this->wheelDiameter  = val; break;
    case 17: this->autoTurnoffTime  = val; break; 
  }
}

uint16_t RemoteSettings::getValue( uint8_t index )
{
  uint16_t value;
  
  switch(index){
    case 0: value = this->mode; break;
    case 1: value = this->displayRotate; break;
    case 2: value = this->upperTrigger; break;
    case 3: value = this->lowerTrigger; break;
    case 4: value = this->throttleMin; break;
    case 5: value = this->throttleCenter; break;
    case 6: value = this->throttleMax; break;
    case 7: value = this->throttleDeadzone; break;
    case 8: value = this->throttleLimit; break;
    case 9: value = this->brakeLimit; break;
    case 10: value = this->batteryCells; break;
    case 11: value = this->batteryMinVoltage; break;
    case 12: value = this->batteryMaxVoltage; break;
    case 13: value = this->motorPoles; break;
    case 14: value = this->pulleyMotor; break;
    case 15: value = this->pulleyWheel; break;
    case 16: value = this->wheelDiameter; break;
    case 17: value = this->autoTurnoffTime; break;
  }

  return value;
}

bool RemoteSettings::inRange(uint16_t value, uint8_t index)
{
  return ((SETTINGS_RULES[index][1] <= value) && (value <= SETTINGS_RULES[index][2]));
}
