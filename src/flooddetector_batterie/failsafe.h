#ifndef FAILSAFE_H
#define FAILSAFE_H

#include <stdint.h>

namespace failsafe{

#define MAX_TIME_VALUE 0xFFFFFFFF

class FailSafe{
  public:
    FailSafe(const unsigned long maxTime = 86400);

    void set_maxTime(unsigned long maxTime);
    void updateTime(unsigned long currTime);
    void resetTime(unsigned long currTime);
   
  private:
    unsigned long _maxTime;
    unsigned long _startTime;
    unsigned long _currTime;
    
    void(* resetFunc) (void) = 0; //declare reset function @ address 0

};
}
#endif
