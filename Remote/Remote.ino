#include "Remote.h"

Remote Remote;

void setup() {
  
  Serial.begin(9600);
  Remote.begin();
  
}

void loop() {

  Remote.calculateThrottle();

  // Serial.println(Remote.getThrottle());

  if( millis() < Remote.startupTimer && Remote.upperTrigger() ) {

    // Enter Settings
    Remote.changeSettings = true;
   
  }

  if(Remote.remoteSettings){

    // Settings mode

    Remote.menuLoop();
    
  }else{

    // Normal mode
    
  }
  
  Remote.Display.update();

}


