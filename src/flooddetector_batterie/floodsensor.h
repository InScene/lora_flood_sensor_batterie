#ifndef FLOODSENSOR_H
#define FLOODSENSOR_H

#include <stdint.h>

namespace floodsensor {
class FloodSensor {

  public:

    FloodSensor(const uint8_t pin);
    void init();
    void doFloodDetection();
    bool isFloodDetected() const;

  private:
  
    uint8_t _pin;
    bool _floodDetected;
    uint8_t _detectCnt;
    const uint8_t _detectThresholdMin = 0;
    const uint8_t _detectThresholdMax = 3;

    void updateDetectCnt(const int currSensorVal);
    void updateFloodDetecedVal();
    
};
}
#endif
