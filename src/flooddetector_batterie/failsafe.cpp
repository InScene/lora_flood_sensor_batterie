#include "failsafe.h"
#include <Arduino.h>

using namespace failsafe;

FailSafe::FailSafe(const unsigned long maxCnt) : 
  _maxCount(maxCnt),
  _cnt(0),
  RESET_INTERVAL(25000),
  RESET_CNT(0),
  LOOP_MAX_CNT(2500000),
  LOOP_CNT(0),
  WDT_LOOP_MAX_CNT(2500000),
  WDT_LOOP_CNT(0) {
  
}

void FailSafe::increaseCnt() {
  if (++_cnt >= _maxCount) {
    _cnt = 0;
    #ifdef ACTIVATE_PRINT
      Serial.println(F("Reset device !!!"));
      Serial.flush(); // give the serial print chance to complete
    #endif
    resetFunc();  //call reset
  }
}

void FailSafe::resetCnt(){
  _cnt = 0;
}

void FailSafe::resetWhenCounterMaxReached() {
  RESET_CNT++;
  if (RESET_CNT >= RESET_INTERVAL) {
    RESET_CNT = 0;
    #ifdef ACTIVATE_PRINT
      Serial.println(F("Reset device"));
      Serial.flush(); // give the serial print chance to complete
    #endif
    //resetFunc();  //call reset
  } else {
    #ifdef ACTIVATE_PRINT
      Serial.print(F("Reset counter: "));
      Serial.print(RESET_CNT);
      Serial.print(F(", from:"));
      Serial.println(RESET_INTERVAL);
      Serial.flush(); // give the serial print chance to complete
    #endif
  }
}

void FailSafe::resetWhenLoopCounterMaxReached() {
  LOOP_CNT++;
  if(LOOP_CNT % LOOP_MAX_CNT == 0 && LOOP_CNT != 0) {
    #ifdef ACTIVATE_PRINT
      Serial.println(F("Reset device because of loop overrun"));
      Serial.flush(); // give the serial print chance to complete
    #endif
    LOOP_CNT = 0;
    //resetFunc();  //call reset
  }
}

bool FailSafe::failsaveWdtEndlessLoopInterrupter(){
    WDT_LOOP_CNT++;
    if(WDT_LOOP_CNT >= WDT_LOOP_MAX_CNT) {
      #ifdef ACTIVATE_PRINT
        Serial.print(F("WDT error. Loop counter max reached!"));
        Serial.println(WDT_LOOP_CNT);
        Serial.flush(); // give the serial print chance to complete
      #endif
      
      return true;
    }
    return false;
}

void FailSafe::loopOk() {
  LOOP_CNT = 0;
}

void FailSafe::wdtOk() {
  WDT_LOOP_CNT = 0;
}
