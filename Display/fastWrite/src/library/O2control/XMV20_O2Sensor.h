#ifndef XMV20_O2Sensor_h
#define XMV20_O2Sensor_h

#define O2Sensor_eepromAddr 0xa9
/*  0: percent19[0]
 *  1: percent19[1]
 *  2: percent100[0]
 *  3: percent100[1]
 */

// #define O2Sensor_defaultPercent19  236
#define O2Sensor_defaultPercent19  218
// #define O2Sensor_defaultPercent100 955
#define O2Sensor_defaultPercent100 1023

class O2Sensor {
public:
  O2Sensor(int);
  void init(void);
  void readADC(void);
  int getValue(void);
  void setReference(int,int);
  void setReference_temporary(int,int);

private:
  int sensePin;
  int percent19;
  int percent100;
  float percent19f;
  float percent100f;
  int adcDiff;
  int buffer[256];
  int bufferPos;
  long accm;
  bool bufferFilled;
};

#endif
