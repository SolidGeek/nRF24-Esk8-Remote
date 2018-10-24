#include "Remote.h"

Remote Remote;

void setup() {
  
  Serial.begin(9600);
  Remote.begin();
  
}

void loop() {

  if( millis() < Remote.startupTimer && Remote.upperTrigger() ) {

    // Enter Settings
    Remote.changeSettings = true;
   
  }
  
  Remote.Display.update();

  // Serial.println( Remote.getThrottle() );
  
}


