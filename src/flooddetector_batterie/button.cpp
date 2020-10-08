#include "button.h"
#include <Arduino.h>

using namespace button;

bool Button::_isPressed = false;
unsigned long Button::_lastInterruptTime = 0;

Button::Button(){
  _isPressed = false;
  _lastInterruptTime = 0;
}
  
void Button::pressed() {
  cli();
  unsigned long interruptTime = micros();  // If interrupts come faster than 50ms, assume it's a bounce and ignore
  if (interruptTime - _lastInterruptTime > 50) 
  {
    _isPressed = true;
    _lastInterruptTime = interruptTime;
  }
  sei();
}

void Button::init() {
  // Set the interrupt pin as input pin
  pinMode(button_int_Pin, INPUT);
  // "On rising edge on the interrupt pin" --> "Execute the ISR
  attachInterrupt(digitalPinToInterrupt(button_int_Pin), pressed, RISING);
}

bool Button::isPressed() {
  return _isPressed;
}

void Button::reset() {
  _isPressed = false;
}
