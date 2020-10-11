#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

namespace battery{

#define battery_Adc_Pin A0

class Battery{
  public:
    Battery(const uint8_t pinOnOff, const uint8_t pinAdc);
    void init();
    void fetchData();
    float getVoltage();
    void print();
    
  private:
    const uint8_t _pinOnOff;
    const uint8_t _pinAdc;
    uint16_t _adcValue;

    void switchMeasurementOn();
    void switchMeasurementOff();
};
}
#endif
