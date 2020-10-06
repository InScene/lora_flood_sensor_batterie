#ifndef FAILSAFE_H
#define FAILSAFE_H

#include <stdint.h>

namespace failsafe{

#define ACTIVATE_PRINT 1

class FailSafe{
  public:
    FailSafe(const unsigned long maxCnt);

    void increaseCnt();
    void resetCnt();
    
    void resetWhenCounterMaxReached();
    void resetWhenLoopCounterMaxReached();
    bool failsaveWdtEndlessLoopInterrupter();
    void loopOk();
    void wdtOk();
    
  private:
    const unsigned long _maxCount;
    unsigned long _cnt;
    
    // Resette Device jeden Tag
    const unsigned RESET_INTERVAL;
    unsigned RESET_CNT;
    
    // Resette Device, wenn Loop Cnt max erreicht. Passiert nur im Fehlerfall.
    const unsigned long LOOP_MAX_CNT; // Das sind ca. 5 Min
    unsigned long LOOP_CNT;
    
    // Sicherstellen, dass die WDT Sleep Schleife im Fehlerfall unterbrochen wird
    const unsigned long WDT_LOOP_MAX_CNT;
    unsigned long WDT_LOOP_CNT;

    void(* resetFunc) (void) = 0; //declare reset function @ address 0

};
}
#endif
