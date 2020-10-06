#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

namespace battery{

#define battery_Adc_Pin A0

class Battery{
  public:
    Battery();
    void init();
    void fetchData();
    float getVoltage();
    void print();
    
  private:
    uint16_t adcValue;
};
}
#endif
