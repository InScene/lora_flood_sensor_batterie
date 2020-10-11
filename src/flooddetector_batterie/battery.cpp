#include "battery.h"
#include <Arduino.h>

using namespace battery;

Battery::Battery(const uint8_t pinOnOff, const uint8_t pinAdc) : 
  _pinOnOff(pinOnOff),
  _pinAdc(pinAdc),
  _adcValue(0) {
  
}

void Battery::init() {
  analogReference(INTERNAL);
  pinMode(_pinOnOff, OUTPUT);
  switchMeasurementOn();
  delay(1000);
  // Read battery data three times to let adc oscillate in
  analogRead(_pinAdc);
  analogRead(_pinAdc);
  analogRead(_pinAdc);
  switchMeasurementOff();
}

void Battery::fetchData() {
  switchMeasurementOn();
  delay(500);
  
  uint32_t val = analogRead(_pinAdc);
  val += analogRead(_pinAdc);
  val += analogRead(_pinAdc);
  
  switchMeasurementOff();
  
  _adcValue = val/3;
}

float Battery::getVoltage() {
  return _adcValue * 0.004333333;
}

void Battery::print() {
  Serial.print(F("Battery voltage: "));
  Serial.println(getVoltage());
}

void Battery::switchMeasurementOn() {
  digitalWrite(_pinOnOff, LOW);
}

void Battery::switchMeasurementOff() {
  digitalWrite(_pinOnOff, HIGH);
}
