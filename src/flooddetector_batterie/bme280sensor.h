#ifndef BME280SENSOR_H
#define BME280SENSOR_H

namespace bme280_sensor{
  
#define BME280_SDA_PIN A4
#define BME280_SCL_PIN A5
#define BME280_VCC_PIN 7

class BME280Sensor {
  public:
    BME280Sensor();
    void init();
    bool fetchData();
    bool fetchDataOnlyTempHum();

    float getTemperature();
    float getHumidity();
    float getPressure();

    void print();
    
  private:
    float _temperature;
    float _humidity;
    float _pressure;

    void initReading();
    void deinitReading();
    bool startForcedReading();
    bool startForcedReadingWithoutPressure();
    bool readTemperature();
    bool readHumidity();
    bool readPressure();
};
}
#endif
