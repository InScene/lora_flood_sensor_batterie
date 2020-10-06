#ifndef DATASTORAGE_H
#define DATASTORAGE_H

#include <stdint.h>

namespace datastorage {
class DataStorage{
  public:
    DataStorage();

    void init();

    uint16_t get_rainsenseCloudburstBorder();
    uint16_t get_rainsenseHeavyRainBorder();
    uint16_t get_rainsenseLightRainBorder();

    void set_rainsenseCloudburstBorder(uint16_t val);
    void set_rainsenseHeavyRainBorder(uint16_t val);
    void set_rainsenseLightRainBorder(uint16_t val);

    void persist();
    void print();
    bool isValid();
    
  private:
    const uint8_t _crcAdr = 0;
    const uint8_t _cloudBurstAdr = 4;
    const uint8_t _heavyRainAdr = 6;
    const uint8_t _lightRainAdr = 8;
    
    uint16_t _cloudburst;
    uint16_t _heavyRain;
    uint16_t _lightRain;

    void readValues();
    void writeInt(uint16_t adr, uint16_t val);
    uint16_t readInt(uint16_t adr);
    void writeLong(uint16_t adr, unsigned long val);
    unsigned long readLong(uint16_t adr);
    unsigned long calculate_crc(void);
};
}

#endif
