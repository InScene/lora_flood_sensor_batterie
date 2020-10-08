#include "floodsensor.h"
#include <Arduino.h>

using namespace floodsensor;

FloodSensor::FloodSensor(const uint8_t pin) :
  _pin(pin),
  _floodDetected(false),
  _detectCnt(_detectThresholdMin) {
}
 
void FloodSensor::init() {
  pinMode(_pin, INPUT);
}

void FloodSensor::doFloodDetection() {
  updateDetectCnt(digitalRead(_pin));
  updateFloodDetecedVal();
}

bool FloodSensor::isFloodDetected() const {
  return _floodDetected;
}

void FloodSensor::updateDetectCnt(const int currSensorVal) {
  if(currSensorVal == HIGH) {
    (_detectCnt < _detectThresholdMax) ? _detectCnt++ : 0;
  } else {
    (_detectCnt > _detectThresholdMin) ? _detectCnt-- : 0;
  }

  Serial.print(F("Flood detect cnt: "));
  Serial.println(_detectCnt);
  Serial.flush();
}

void FloodSensor::updateFloodDetecedVal() {
  if(_detectCnt >= _detectThresholdMax) {
    _floodDetected = true;
    return true;
    
  } else if(_detectCnt == _detectThresholdMin){
    _floodDetected = false;
    return false;
  }
}
