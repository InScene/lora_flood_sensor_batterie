#ifndef DATASTORAGE_H
#define DATASTORAGE_H

#include <stdint.h>

namespace datastorage {
class DataStorage{
  public:
    DataStorage();

    void init();

    uint16_t get_sendInterval();
    uint16_t get_highTempThreshold();

    void set_sendInterval(uint16_t val);
    void set_highTempThreshold(uint16_t val);

    void persist();
    void print();
    bool isValid();
    
  private:
    const uint8_t _crcAdr = 0;
    const uint8_t _sendIntvAdr = 4;
    const uint8_t _highTempThresholdAdr = 6;
    
    uint16_t _sendInterval;
    uint16_t _highTempThreshold;

    void readValues();
    void writeInt(uint16_t adr, uint16_t val);
    uint16_t readInt(uint16_t adr);
    void writeLong(uint16_t adr, unsigned long val);
    unsigned long readLong(uint16_t adr);
    unsigned long calculate_crc(void);
};
}

#endif
