#ifndef TIME_H
#define TIME_H

#include <stdint.h>

namespace time {

class Time {

  public:

    Time(const uint8_t secDuringSleep);
    void reset();
    void update(const uint16_t sleepCycles=0);
    unsigned long get_currentTime() const;
    unsigned long Time::calculateNextCyclicSendTime(const unsigned long lastCyclicSendTime, uint16_t txInterval) const;
    bool Time::cyclicSleepTimeOver(const unsigned long lastCyclicSendTime, const unsigned long nextCyclicSendTime) const;
    
  private:

    unsigned long _msecDuringSleep;
    unsigned long _currTime;
    unsigned long _lastCyclicSend = 0;
    unsigned long _lastTimeStored = 0;
    
};
}
#endif
