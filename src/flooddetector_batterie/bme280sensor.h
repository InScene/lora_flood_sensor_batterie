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

    float getTemperature();
    float getHumidity();
    float getPressure();

    void print();
    
  private:
    float temperature;
    float humidity;
    float pressure;

    bool startReading();
    bool readTemperature();
    bool readHumidity();
    bool readPressure();
};
}
#endif
