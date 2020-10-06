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
 *  Receive raw little endian rain sense border values:
 *   Example: FF 00 90 01 9C 02
 *   FF 00 : 256 = cloudburst
 *   90 01 : 400 = heavy rain
 *   9c 02 : 668 = light rain
 *   
 * ToDo:
 * Set keys in radio_keys.h (value from staging.thethingsnetwork.com)
 *
 *******************************************************************************/
#include <lmic.h>
#include <hal/hal.h>

// use low power sleep; comment next line to not use low power sleep
#include "LowPower.h"
#include <CayenneLPP.h>
#include "bme280sensor.h"
#include "battery.h"
#include "failsafe.h"
#include "datastorage.h"

// Sende alle 10 Minuten eine Nachricht
const unsigned TX_INTERVAL = 600;

bme280_sensor::BME280Sensor g_bmeSensor;
battery::Battery g_battery;
failsafe::FailSafe g_resetDaily(86400 / TX_INTERVAL);
failsafe::FailSafe g_wdtFailSafe(1000);
failsafe::FailSafe g_loopFailSafe(2500000);
datastorage::DataStorage g_dataStorage;

bool next = false;
#define ACTIVATE_PRINT 1

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
            break;
        case EV_JOINED:
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_TXCOMPLETE:
            if(LMIC.dataLen) { // data received in rx slot after tx
              handleRxData();
            }
            // Schedule next transmission
            next = true;            
            break;
        case EV_RXCOMPLETE:
            Serial.println(F("Event rx complete"));
            // Do nothing
            break;
        default:
            // If this happends, no connection possible. 
            #ifdef ACTIVATE_PRINT
              Serial.print(F("Not Handled event. OnEvent: "));
              Serial.println(ev);
            #endif
            break;
    }    
}

void handleRxData() {
//  if(LMIC.dataLen==6){
//    Serial.println(F("Rx data with correct length received"));
//    uint16_t rxData[3];
//    memcpy(rxData, LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
//
//    g_dataStorage.persist();
//    g_dataStorage.print();
//    useStoredValues();
//  } else {
//    #ifdef ACTIVATE_PRINT
//      Serial.print(F("No correct rx data length received. Len:"));
//      Serial.println(LMIC.dataLen);
//    #endif
//  }
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
        Serial.println(F("Fetch data"));
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
  if(g_bmeSensor.fetchData()) {
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
 

  return lpp;
}


void useStoredValues() {
//  g_rainSense.set_cloudburst(g_dataStorage.get_rainsenseCloudburstBorder());
//  g_rainSense.set_heavyRain(g_dataStorage.get_rainsenseHeavyRainBorder());
//  g_rainSense.set_lightRain(g_dataStorage.get_rainsenseLightRainBorder());
}


void setup() {
    #ifdef ACTIVATE_PRINT
      Serial.begin(9600);
      Serial.println(F("Enter setup"));
    #endif

    g_dataStorage.init();
    g_dataStorage.print();
    useStoredValues();
    
    g_bmeSensor.init();
    g_battery.init();    

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



void sleepForATime() {
  const int sleepcycles = TX_INTERVAL / 8;  // calculate the number of sleepcycles (8s) given the TX_INTERVAL
  #ifdef ACTIVATE_PRINT
    Serial.print(F("Enter sleeping for "));
    Serial.print(sleepcycles);
    Serial.println(F(" cycles of 8 seconds"));
    Serial.flush(); // give the serial print chance to complete
  #endif
  
  for (int i=0; i<sleepcycles; i++) {
    g_wdtFailSafe.resetCnt();
    // Enter power down state for 8 s with ADC and BOD module disabled
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    // If watchdog not triggered, go back to sleep. End sleep by other interrupt 
     while(!LowPower.isWdtTriggered())
    {
      g_wdtFailSafe.increaseCnt();
      LowPower.returnToSleep();
    }
  }
   
  #ifdef ACTIVATE_PRINT
    Serial.println("******************* Sleep complete *******************");
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
    next = false;
  
    // Start job
    do_send(&sendjob);
  }

  g_loopFailSafe.increaseCnt();
}
