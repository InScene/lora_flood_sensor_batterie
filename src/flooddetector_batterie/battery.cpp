#include "battery.h"
#include <Arduino.h>

using namespace battery;

Battery::Battery() : adcValue(0) {
  
}

void Battery::init() {
  analogReference(INTERNAL);
  delay(1000);
  // Read battery data three times to let adc oscillate in
  analogRead(battery_Adc_Pin);
  analogRead(battery_Adc_Pin);
  analogRead(battery_Adc_Pin);
}

void Battery::fetchData() {
  uint32_t val = analogRead(battery_Adc_Pin);
  val += analogRead(battery_Adc_Pin);
  val += analogRead(battery_Adc_Pin);

  adcValue = val/3;
}

float Battery::getVoltage() {
  return adcValue * 0.004333333;
}

void Battery::print() {
  Serial.print(F("Battery voltage: "));
  Serial.println(getVoltage());
}
