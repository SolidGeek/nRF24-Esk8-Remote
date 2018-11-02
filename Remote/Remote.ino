#include "Remote.h"

Remote Remote;

bool triggerFlag = false;
bool settingFlag = false;

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

  if(Remote.changeSettings){

    // Settings mode

    if(Remote.upperTrigger()){
      if(triggerFlag == false){
        Remote.selectSetting = !Remote.selectSetting;
        triggerFlag = true;
      }
    }else{
      triggerFlag = false;
    }

    if( Remote.getThrottle() > THROTTLE_CENTER + 200 ){
      // Add one to the selected setting or move a setting up  
      if( settingFlag == false && Remote.currentSetting > 0 ){
        Remote.currentSetting--;
        settingFlag = true;  
      }
    }else if( Remote.getThrottle() < THROTTLE_CENTER - 200){
      // Substract one to the selected setting or move a setting down
      if( settingFlag == false && Remote.currentSetting < SETTINGS_COUNT ){
        Remote.currentSetting++;
        settingFlag = true;  
      }
    }else if( Remote.getThrottle() >= THROTTLE_CENTER - 50 && Remote.getThrottle() <= THROTTLE_CENTER + 50){
      settingFlag = false;  
    }
    
  }else{

    // Normal mode
    
  }
  
  Remote.Display.update();

  // Serial.println( Remote.getThrottle() );
  
}


