#include "Remote.h"

Remote Remote;

void setup() {
  
  Serial.begin(9600);
  Remote.begin();
  
}

void loop() {
  
  // Remote.Display.update();

  Serial.println( Remote.getThrottle() );
  
}


