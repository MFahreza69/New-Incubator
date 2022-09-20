
#ifndef XMV20_O2Valve_h
#define XMV20_O2Valve_h

#define O2Valve_minDuration   1000UL
#define O2Valve_minOnDuration  150UL

#define O2Valve_eepromAddr 0x0
/*  0: duration[0]
 *  1: duration[1]
 *  2: duration[2]
 *  3: duration[3]
 *  4: onDuration[0]
 *  5: onDuration[1]
 *  6: onDuration[2]
 *  7: onDuration[3]
 */

class O2Valve {
public:
  O2Valve(int);
  void init(void);
  void refresh(void);
  void setDutyCycle(int);

private:
  int valvePin;
  bool valveState;
  unsigned long lastCycle;
  int targetO2;
  unsigned long duration;
  unsigned long onDuration;
};

#endif
