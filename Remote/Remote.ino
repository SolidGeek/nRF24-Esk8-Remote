#include "Remote.h"

Remote Remote;

void setup() {
  Serial.begin(9600);

  Remote.begin();
}

void loop() {
  
  Remote.display.update();

  delay(100);
  
}


