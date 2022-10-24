#ifndef Xbaby_h
#define Xbaby_h

class Xbaby {
  public:
    Xbaby();
    void write_eeprom(int address, int number, float value_data);
    int get_value_pos(int set_humidity, int value_humidity, int last_pos);
    float get_value_baby_skin(float valueA, float valueB, int pin);
    int get_value_fan(float set_temp,int mode, float temp_chamber, float temp_baby0, float temp_baby1, float set_baby_humidity, float baby_humidity);
    int get_value_heater(float set_temp,int mode, int high, float temp_chamber, float temp_baby0, float temp_baby1);

  private:
    int pos;
    int fan;
    int heater;
    float all_temp;
    int pos_setup;
    unsigned long tcal;
    int sampleData[21];
};

#endif