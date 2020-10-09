/*******************************************************************************
 * Copyright (c) 2020 by Christian Mertens
 *
 * A batterie powered flood sensor which used LoRaWAN to send the data to the Things Network server.
 * The hardware is based on an Arduino mini. 
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Send following data via lpp:
 *  1: Temperature
 *  2: Relative humidity
 *  4. Battery value (Volt)    
 *  5. Flood sensor: 0=dry, 1=water detected
 *  6. Test send: 0=no test, 1=test message
 *  
 *  Receive raw little endian values:
 *   Example: BE 00 32 00
 *   BE 00 : 190 = send interval set to 190 seconds
 *   32 00 : 50 = high temperature threshold set to 50°
 *   
 * ToDo:
 * Set keys in radio_keys.h (value from staging.thethingsnetwork.com)
 *
 *******************************************************************************/
#include <lmic.h>
#include <hal/hal.h>

#define ACTIVATE_PRINT 1

#include "LowPower.h"
#include <CayenneLPP.h>
#include "bme280sensor.h"
#include "battery.h"
#include "failsafe.h"
#include "datastorage.h"
#include "button.h"
#include "floodsensor.h"
#include "led.h"

bme280_sensor::BME280Sensor g_bmeSensor;
battery::Battery g_battery;
failsafe::FailSafe g_resetDaily;
failsafe::FailSafe g_wdtFailSafe(1000);
failsafe::FailSafe g_loopFailSafe(2500000);
datastorage::DataStorage g_dataStorage;
button::Button g_button;
floodsensor::FloodSensor g_floodSensor(8);
led::Led g_led(9);

enum sendMode {
  cyclic = 0,
  flood = 1,
  highTemp = 2,
  test = 3
};

const uint8_t g_secondsDuringSleep = 8;
const uint8_t g_floodCheckSleepCycles = (32 / g_secondsDuringSleep); // Flood check sleep cycles (seconds in multiples of 8 / seconds during one sleep cycle)
const uint8_t g_tempCheckSleepCycles = (120 / g_secondsDuringSleep); // Temp check sleep cycles (seconds in multiples of 8 / seconds during one sleep cycle)

unsigned g_txIntervall = 0;

uint8_t g_sendMode = sendMode::cyclic;
bool g_isFloodMsgAck = false;
uint8_t g_highTempThreshold = 0;

bool next = false;


/****************************************** LoRa *******************************/
#include "radio_keys.h"

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {A3, 3, 4},
};


void onEvent (ev_t ev) {
    switch(ev) {
        case EV_JOINING:
            // Do noting, because we react on EV_JOINED
            #ifdef ACTIVATE_PRINT
              Serial.println(F("Event joining"));
            #endif
            break;
        case EV_JOINED:
            #ifdef ACTIVATE_PRINT
              Serial.println(F("Event joined"));
            #endif
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_TXCOMPLETE:
            #ifdef ACTIVATE_PRINT
              Serial.println(F("Event txcomplete"));
            #endif
            if(sendMode::flood == g_sendMode) {
              g_isFloodMsgAck = true;
             
            } else if(sendMode::cyclic == g_sendMode) {
              if(!g_floodSensor.isFloodDetected()) {
                g_isFloodMsgAck = false;
              }
              
            }
            
            if(LMIC.dataLen) { // data received in rx slot after tx
              handleRxData();
            }
            // Schedule next transmission
            next = true;            
            break;
        case EV_RXCOMPLETE:
            #ifdef ACTIVATE_PRINT
              Serial.println(F("Event rx complete"));
            #endif
            // Do nothing
            break;
        default:
            // If this happends, no connection possible. 
            #ifdef ACTIVATE_PRINT
              Serial.print(F("Not Handled event. OnEvent: "));
              Serial.println(ev);
            #endif
            if(sendMode::test == g_sendMode) {
              g_led.flashing4Times();
            }
            break;
    }    
}

void handleRxData() {
  if(LMIC.dataLen==4){
    #ifdef ACTIVATE_PRINT
      Serial.println(F("Rx data with correct length received"));
    #endif
    uint16_t rxData[2];
    memcpy(rxData, LMIC.frame + LMIC.dataBeg, LMIC.dataLen);

    uint16_t sendInterval = rxData[0];
    uint16_t highTempThreshold = rxData[1];

    // Send interval can only be between 190 seconds and 18 h 12,5 minutes
    if(sendInterval >= 190 && sendInterval <= 0xffff) {
      g_dataStorage.set_sendInterval(sendInterval);
    }

    // High temperature threshold can only be between 1°C and 84°C
    if(highTempThreshold >= 1 && highTempThreshold <= 84) {
      g_dataStorage.set_highTempThreshold(highTempThreshold);
    }
    
    g_dataStorage.persist();
    #ifdef ACTIVATE_PRINT
      g_dataStorage.print();
    #endif
    useStoredValues();
    
  } else {
    #ifdef ACTIVATE_PRINT
      Serial.print(F("No correct rx data length received. Len:"));
      Serial.println(LMIC.dataLen);
    #endif
  }
}

void do_send(osjob_t* j){
    #ifdef ACTIVATE_PRINT
      Serial.println(F("Enter do_send"));
    #endif
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      #ifdef ACTIVATE_PRINT
        Serial.println(F("OP_TXRXPEND, not sending"));
      #endif      
    } else {
      #ifdef ACTIVATE_PRINT
        Serial.print(F("Fetch data in send mode: "));
        Serial.println(g_sendMode);
      #endif
      CayenneLPP lpp = getCayenneFormatedData();
      
      /**** send data *****/
      LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);

      #ifdef ACTIVATE_PRINT
        Serial.println(F("Packet queued"));
      #endif
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

CayenneLPP getCayenneFormatedData() {
  /* Initialize CayenneLPP with max buffer size */
  CayenneLPP lpp(30);

  /**** get BME280 Data ****/      
  if(g_bmeSensor.fetchDataOnlyTempHum()) {
    lpp.addTemperature(1, g_bmeSensor.getTemperature());
    lpp.addRelativeHumidity(2, g_bmeSensor.getHumidity());
    #ifdef ACTIVATE_PRINT
      g_bmeSensor.print();
    #endif
    
  } else {
    #ifdef ACTIVATE_PRINT
      Serial.println(F("Error getting bme280 data!"));
    #endif
  }

  /**** get battery voltage *****/
  g_battery.fetchData();
  lpp.addAnalogInput(4, g_battery.getVoltage() );
  #ifdef ACTIVATE_PRINT
    g_battery.print();
  #endif

  /**** get flood value *****/
  uint8_t floodVal = 0;
  (g_floodSensor.isFloodDetected()) ? floodVal = 1 : floodVal = 0;
  lpp.addDigitalInput(5, floodVal);

  /**** set test value *****/
  uint8_t testVal = 0;
  (sendMode::test == g_sendMode ) ? testVal = 1 : testVal = 0;
  lpp.addDigitalInput(6, testVal);
 
  return lpp;
}

void useStoredValues() {
  g_txIntervall = g_dataStorage.get_sendInterval();
  g_highTempThreshold = g_dataStorage.get_highTempThreshold();

  g_resetDaily.set_maxCnt(86400 / g_txIntervall);
}


void setup() {
    #ifdef ACTIVATE_PRINT
      Serial.begin(9600);
      Serial.println(F("Enter setup"));
    #endif

    g_dataStorage.init();
    #ifdef ACTIVATE_PRINT
      g_dataStorage.print();
    #endif
    useStoredValues();

    g_bmeSensor.init();
    g_battery.init();
    g_button.init();    
    g_floodSensor.init();
    g_led.init();

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // Aktiviere Interrupts
    interrupts();
    
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // got this fix from forum: https://www.thethingsnetwork.org/forum/t/over-the-air-activation-otaa-with-lmic/1921/36
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    // Start job
    do_send(&sendjob);
    
    #ifdef ACTIVATE_PRINT
      Serial.println(F("Leave setup"));
    #endif
}

bool doHighTempCheck() {
  if(isHighTemperature()) {
    g_sendMode = sendMode::highTemp;
    #ifdef ACTIVATE_PRINT
      Serial.print(F("High temperature detected: "));
      Serial.println(g_bmeSensor.getTemperature());
      Serial.flush(); // give the serial print chance to complete
    #endif
    
    return true;
  }
  return false;
}

bool doFloodCheck() {
  g_floodSensor.doFloodDetection();
  // There is a flood and until now not acknowledged
  if(!g_isFloodMsgAck && g_floodSensor.isFloodDetected()){
    g_sendMode = sendMode::flood;
    #ifdef ACTIVATE_PRINT
      Serial.println(F("Flood detected"));
      Serial.flush(); // give the serial print chance to complete
    #endif
    
    return true;
  } 
  return false;
}

bool doTestButtonCheck() {
  if(g_button.isPressed()){
    g_button.reset();
    g_led.switchOn();
    g_sendMode = sendMode::test;
    #ifdef ACTIVATE_PRINT
      Serial.println(F("Button is pressed"));
      Serial.flush(); // give the serial print chance to complete
    #endif
    
    return true;
  } 
  return false;
}

void sleepForATime() {
  const int sleepcycles = g_txIntervall / g_secondsDuringSleep;  // calculate the number of sleepcycles (g_secondsDuringSleep) given the g_txIntervall
  const int floodCheckCycles = (g_floodCheckSleepCycles <= sleepcycles) ? g_floodCheckSleepCycles : sleepcycles;
  const int tempCheckCycles = (g_tempCheckSleepCycles <= sleepcycles) ? g_tempCheckSleepCycles : sleepcycles;
  #ifdef ACTIVATE_PRINT
    Serial.print(F("Enter sleeping for "));
    Serial.print(sleepcycles);
    Serial.println(F(" cycles of 8 seconds"));
    Serial.flush(); // give the serial print chance to complete
  #endif

  g_sendMode = sendMode::cyclic;
  
  for (int i=1; i<=sleepcycles; i++) {
    g_wdtFailSafe.resetCnt();
    // Enter power down state for 8 s with ADC and BOD module disabled
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    // If watchdog not triggered, go back to sleep. End sleep by other interrupt 
     while(!LowPower.isWdtTriggered())
    {
      g_wdtFailSafe.increaseCnt();
      LowPower.returnToSleep();
    }

    // Flood check
    if((i % floodCheckCycles == 0) && doFloodCheck()) {
      break;
    }

    // Temperature check
    if((i % tempCheckCycles == 0) && doHighTempCheck()) {
        break;
    }

    // Test button check
    if(doTestButtonCheck()){
      break;
    } 

    g_led.switchOff();
  }
   
  #ifdef ACTIVATE_PRINT
    Serial.println("******************* Sleep ended *******************");
  #endif
}

void loop() {
  extern volatile unsigned long timer0_overflow_count;
  
  if (next == false) {
    os_runloop_once();

  } else {
    g_resetDaily.increaseCnt();
    g_loopFailSafe.resetCnt();

    sleepForATime();
    send_data();
  }

  g_loopFailSafe.increaseCnt();
}

void send_data() {
  next = false;
  
  // Start job
  do_send(&sendjob);
}

bool isHighTemperature() {
  if(g_bmeSensor.fetchData() && 
    (g_bmeSensor.getTemperature() >= g_highTempThreshold)) {
      return true;
  }
  return false;
}
