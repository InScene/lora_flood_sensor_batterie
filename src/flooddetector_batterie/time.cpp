#include "time.h"
#include <Arduino.h>

using namespace time;

Time::Time(const uint8_t secDuringSleep) :
  _msecDuringSleep(secDuringSleep*1000),
  _currTime(0),
  _lastCyclicSend(0),
  _lastTimeStored(0){
}
 
void Time::reset() {
  _currTime = 0;
  _lastCyclicSend = 0;
  _lastTimeStored = 0;
}

void Time::update(const uint16_t sleepCycles) {
  const unsigned long offset = ((unsigned long)sleepCycles) * _msecDuringSleep;
  unsigned long measuredTime = millis();
  unsigned long test;
  
  if(measuredTime >= _lastTimeStored) {
    test = measuredTime - _lastTimeStored + offset;
    _currTime += test;
  } else {
    test = (0xFFFFFFFF - _lastTimeStored) + measuredTime + offset;
    _currTime += test;
  }

  _lastTimeStored = measuredTime;
}

unsigned long Time::get_currentTime() const {
  return _currTime;
}

unsigned long Time::calculateNextCyclicSendTime(const unsigned long lastCyclicSendTime, uint16_t txInterval) const {
  unsigned long nextTime;
  const unsigned long txIntervalMs = ((unsigned long)txInterval) * 1000;
  const unsigned long remainingTimeTillOverflow = 0xFFFFFFFF - lastCyclicSendTime;

  if(remainingTimeTillOverflow >= txIntervalMs) {
    nextTime = lastCyclicSendTime + txIntervalMs;
  } else {
    nextTime = txIntervalMs - remainingTimeTillOverflow;
  }

  return nextTime;
}

bool Time::cyclicSleepTimeOver(const unsigned long lastCyclicSendTime, const unsigned long nextCyclicSendTime) const {
  bool timeIsOver = false;
  if(lastCyclicSendTime <= nextCyclicSendTime) {
    if(_currTime >= nextCyclicSendTime) {
      timeIsOver = true;
    }
  } else {
    // This happens becaus millis() has overflow every 50 days
    if(_currTime >= lastCyclicSendTime) {
      // until now, no overflow happend. so time is not over
    } else if(_currTime >= nextCyclicSendTime) {
      timeIsOver = true;
    }
  }

  return timeIsOver;
}
