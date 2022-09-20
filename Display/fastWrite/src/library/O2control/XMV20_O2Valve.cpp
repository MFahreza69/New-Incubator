#include <Arduino.h>
#include <EEPROM.h>
#include "XMV20_O2Valve.h"

O2Valve::O2Valve(int pin){
  valvePin = pin;
  valveState = false;
  lastCycle = 0;
  targetO2 = 0;
  onDuration = 0;
  duration = O2Valve_minDuration;
}

void O2Valve::init(void){
  unsigned long temp = 0;
  for(char i=0; i<4; i++){
    unsigned char val = EEPROM.read(O2Valve_eepromAddr+i);
    temp |= (val << (8*i));
  }
  duration = temp;

  temp = 0;
  for(char i=0; i<4; i++){
    unsigned char val = EEPROM.read(O2Valve_eepromAddr+0x04+i);
    temp |= (val << (8*i));
  }
  onDuration = temp;
  
  pinMode(valvePin, OUTPUT);
  digitalWrite(valvePin, valveState);
}

void O2Valve::refresh(void){
  unsigned long currentTime = millis();
  
  if(valveState){
    // Check for ON Duration
    if(((currentTime - lastCycle) > onDuration) && (onDuration < duration))
      valveState = false;
  } else {
    // Check for cycle duration
    if(((currentTime - lastCycle) > duration) && (onDuration > 0UL)){
      valveState = true;
      lastCycle += duration;
    }
  }

  digitalWrite(valvePin, valveState);
}

void O2Valve::setDutyCycle(int percent){
  if(percent > 100) percent = 100;
  targetO2 = percent;
  
  unsigned int requestedDuration = percent*O2Valve_minDuration/100UL;
  if(requestedDuration == 0){
    onDuration = 0;
    duration = O2Valve_minDuration;
  } else if(requestedDuration < O2Valve_minOnDuration){
    onDuration = O2Valve_minOnDuration;
    duration = (O2Valve_minOnDuration*100UL)/percent;
  } else {
    onDuration = requestedDuration;
    duration = O2Valve_minDuration;
  }

  for(char i=0; i<4; i++){
    EEPROM.update((O2Valve_eepromAddr+i),((duration >> (8*i)) & 0xFF));
    EEPROM.update((O2Valve_eepromAddr+0x04+i),((onDuration >> (8*i)) & 0xFF));
  }
}
