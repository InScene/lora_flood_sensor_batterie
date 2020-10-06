#include "button.h"
#include <Arduino.h>

using namespace button;

bool Button::_isPressed = false;

Button::Button(){
  _isPressed = false;
}
  
void Button::pressed() {
  cli();
  _isPressed = true;
  sei();
}

void Button::init() {
  // Lege den Interruptpin als Inputpin mit Pullupwiderstand fest
  pinMode(button_int_Pin, INPUT_PULLUP);
  // "Bei steigender Flanke auf dem Interruptpin" --> "Führe die ISR aus"
  attachInterrupt(digitalPinToInterrupt(button_int_Pin), pressed, RISING);
}

bool Button::isPressed() {
  return _isPressed;
}

void Button::reset() {
  _isPressed = false;
}
