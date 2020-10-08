#include "led.h"
#include <Arduino.h>

using namespace led;

Led::Led(const uint8_t pin) :
  _pin(pin){
}
 
void Led::init() {
  pinMode(_pin, OUTPUT);
  switchOff();
}

void Led::switchOn() {
  digitalWrite(_pin, HIGH);
}

void Led::switchOff() {
  digitalWrite(_pin, LOW);
}

void Led::flashing4Times() {
  for(int i=0; i<4; i++) {
    digitalWrite(_pin, HIGH);
    delay(200);
    digitalWrite(_pin, LOW);
    delay(200);
  }
}
