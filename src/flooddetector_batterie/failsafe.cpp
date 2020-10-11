#include "failsafe.h"
#include <Arduino.h>

using namespace failsafe;

FailSafe::FailSafe(const unsigned long maxTime) : 
  _maxTime(maxTime),
  _startTime(0),
  _currTime(0){
  
}

void FailSafe::set_maxTime(const unsigned long maxTime) {
  _maxTime = maxTime;
}

void FailSafe::updateTime(const unsigned long currTime) {
  _currTime = currTime;
  
  if(_startTime > _currTime) {
    const unsigned long timeDiffWithOverflow = (MAX_TIME_VALUE - _startTime) + _currTime;
    if(timeDiffWithOverflow >= _maxTime) {
      resetFunc(); 
    }
  } else {
    const unsigned long timeDiff = _currTime - _startTime;
    if(timeDiff >= _maxTime) {
      resetFunc(); 
    }
  }

}

void FailSafe::resetTime(const unsigned long currTime){
  _startTime = currTime;
}
