#ifndef LED_H
#define LED_H

#include <stdint.h>

namespace led {
class Led {

  public:

    Led(const uint8_t pin);
    void init();
    void switchOn();
    void switchOff();
    void flashing4Times();

  private:
  
    uint8_t _pin;
    
    
};
}
#endif
