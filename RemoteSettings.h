
class Settings{

  struct settings{
    uint8_t mode;
    bool displayRotate;
    uint8_t upperTrigger;
    uint8_t lowerTrigger;
    uint16_t throttleMin;
    uint16_t throttleCenter;
    uint16_t throttleMax;
    uint8_t throttleDeadzone;
    uint8_t throttleLimit;
    uint8_t brakeLimit;
    uint8_t batteryCells;
    uint8_t batteryMinVoltage;
    uint8_t batteryMaxVoltage;
    uint8_t motorPoles;
    uint8_t pulleyMotor;
    uint8_t pulleyWheel;
    uint8_t wheelDiameter;
    bool autoTurnoff;
    uint8_t autoTurnoffTime;
  };

  Settings::Settings( void );

  Settings::setDefaultSettings( void );

  Settings::loadSettings( void );

  Settings::updateSettings( void );
  
}
