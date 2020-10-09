#include "bme280sensor.h"
#include <Arduino.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme; // I2C

using namespace bme280_sensor;

BME280Sensor::BME280Sensor() :
  _temperature(0),
  _humidity(0),
  _pressure(0){
  
}

void BME280Sensor::init() {
  pinMode(BME280_VCC_PIN, OUTPUT);
}

bool BME280Sensor::fetchData(){
  bool success = false;
  _temperature = 0;
  _humidity = 0;
  _pressure = 0;
  
  initReading();

  if(startForcedReading()) {
    if(readTemperature() &&
       readHumidity() &&
       readPressure()) {
      success = true;
      
    }
  }

  deinitReading();
  return success;
}

bool BME280Sensor::fetchDataOnlyTempHum() {
  bool success = false;
  _temperature = 0;
  _humidity = 0;
  _pressure = 0;
  
  initReading();

  if(startForcedReadingWithoutPressure()) {
    if(readTemperature() &&
       readHumidity()) {
      success = true;
      
    }
  }

  deinitReading();
  return success;
}

float BME280Sensor::getTemperature() {
  return _temperature;
}

float BME280Sensor::getHumidity() {
  return _humidity;
}

float BME280Sensor::getPressure() {
  return _pressure;
}

void BME280Sensor::initReading() {
  digitalWrite(BME280_VCC_PIN, HIGH);
  delay(1000);
}

void BME280Sensor::deinitReading() {
  digitalWrite(BME280_VCC_PIN, LOW); 
}

bool BME280Sensor::startForcedReading() {
  if(bme.begin(0x76)) {
    // humidity sensing
    // forced mode, 1x temperature / 1x humidity / 1x pressure oversampling
    // filter off
    // suggested rate is 1Hz (1s)
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1,   // temperature
                    Adafruit_BME280::SAMPLING_X1,   // pressure
                    Adafruit_BME280::SAMPLING_X1,   // humidity
                    Adafruit_BME280::FILTER_OFF );
                    
    bme.takeForcedMeasurement();
    
    return true;
  }

  return false;
}

bool BME280Sensor::startForcedReadingWithoutPressure() {
  if(bme.begin(0x76)) {
    // humidity sensing
    // forced mode, 1x temperature / 1x humidity / 0x pressure oversampling
    // pressure off, filter off
    // suggested rate is 1Hz (1s)
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1,   // temperature
                    Adafruit_BME280::SAMPLING_NONE, // pressure
                    Adafruit_BME280::SAMPLING_X1,   // humidity
                    Adafruit_BME280::FILTER_OFF );
              
    bme.takeForcedMeasurement();
    
    return true;
  }

  return false;
}

void BME280Sensor::print() {
  Serial.print(F("BME280 temp: "));
  Serial.print(_temperature);
  Serial.print(F(", hum: "));
  Serial.print(_humidity);
  Serial.print(F(", press: "));
  Serial.println(_pressure);
}

bool BME280Sensor::readTemperature() {
  float val = bme.readTemperature();
  if(val != NAN) {
    _temperature = val;
    return true;
  }
  
  return false;
}

bool BME280Sensor::readHumidity() {
  float val = bme.readHumidity();
  if(val != NAN) {
    _humidity = val;
    return true;
  }

  return false;
}

bool BME280Sensor::readPressure() {
  float val = bme.readPressure();
  if(val != NAN) {
    _pressure = val / 100.0F;
    return true;
  }
  
  return false;
}
