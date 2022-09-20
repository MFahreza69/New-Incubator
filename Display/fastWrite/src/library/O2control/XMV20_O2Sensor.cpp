#include <Arduino.h>
#include <EEPROM.h>
#include "XMV20_O2Sensor.h"

O2Sensor::O2Sensor(int pin){
  sensePin   = pin;
  percent19  = O2Sensor_defaultPercent19;
  percent100 = O2Sensor_defaultPercent100;
  adcDiff    = (O2Sensor_defaultPercent100 - O2Sensor_defaultPercent19);
  bufferPos  = 0;
  accm       = 0;
  bufferFilled = false;
}

void O2Sensor::init(void){
  int temp = 0;
  for(char i=0; i<2; i++){
    unsigned char val = EEPROM.read(O2Sensor_eepromAddr+i);
    temp |= (val << (8*i));
  }
  percent19f = 0.5*1024/3.3;
  percent19 = temp;//(int)percent19f;//temp;

  temp = 0;
  for(char i=0; i<2; i++){
    unsigned char val = EEPROM.read(O2Sensor_eepromAddr+0x02+i);
    temp |= (val << (8*i));
  }
  percent100f = 2.5*1024/3.3;
  percent100 = temp;//(int)percent100f;//temp;
  
  adcDiff = percent100 - percent19;
  
  for(int i=0; i<256; i++)
    buffer[i] = 0;
  bufferPos = 0;
  accm = 0;
  bufferFilled = false;

//  Serial.print("19: ");Serial.println(percent19);
//  Serial.print("100: ");Serial.println(percent100);
  pinMode(sensePin, INPUT);
}

void O2Sensor::readADC(void){
  // read the input on analog pin:
  int sensorValue = analogRead(sensePin);
  
  accm -= buffer[bufferPos];
  accm += sensorValue;
  buffer[bufferPos] = sensorValue;
  if(bufferPos == 255) bufferFilled = true;
  bufferPos = (bufferPos == 255) ? 0 : bufferPos+1;
}

int O2Sensor::getValue(void){
  if(bufferFilled){
    long bufferedVal = accm / 256;
    long percent = ((96-19)*(bufferedVal-(long)percent19)/(long)adcDiff) + 19;
    return percent;
  } else
    return -1;
}

void O2Sensor::setReference(int in19, int in100){
  for(char i=0; i<2; i++){
    EEPROM.update((O2Sensor_eepromAddr+i),((in19 >> (8*i)) & 0xFF));
    EEPROM.update((O2Sensor_eepromAddr+0x02+i),((in100 >> (8*i)) & 0xFF));
  }
  percent19  = in19;
  percent100 = in100;
  adcDiff = percent100 - percent19;
}
void O2Sensor::setReference_temporary(int in19, int in100){
  percent19  = in19;
  percent100 = in100;
  adcDiff = percent100 - percent19;
}
