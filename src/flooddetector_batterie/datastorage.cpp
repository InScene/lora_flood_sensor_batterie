#include "datastorage.h"
#include <Arduino.h>
#include <EEPROM.h>

using namespace datastorage;

DataStorage::DataStorage() :
  _cloudburst(256),
  _heavyRain(400),
  _lightRain(668) {
}

void DataStorage::init() {
  if(isValid()) {
    readValues();
    
  } else {
    Serial.println(F("Eeprom crc error. Use default values."));
  }
}

uint16_t DataStorage::get_rainsenseCloudburstBorder() {
  return _cloudburst;
}

uint16_t DataStorage::get_rainsenseHeavyRainBorder() {
  return _heavyRain;
}

uint16_t DataStorage::get_rainsenseLightRainBorder() {
  return _lightRain;
}

void DataStorage::set_rainsenseCloudburstBorder(uint16_t val) {
  _cloudburst = val;
}

void DataStorage::set_rainsenseHeavyRainBorder(uint16_t val) {
  _heavyRain = val;
}

void DataStorage::set_rainsenseLightRainBorder(uint16_t val) {
  _lightRain = val;
}

void DataStorage::persist() {
  unsigned long old_crc = calculate_crc();
  
  writeInt(_cloudBurstAdr, _cloudburst); 
  writeInt(_heavyRainAdr, _heavyRain);
  writeInt(_lightRainAdr, _lightRain);

  unsigned long crc = calculate_crc();
  writeLong(_crcAdr, crc);

  if(!isValid()) {
    Serial.print(F("Eeprom error during persisting values. calcCrc:"));
    Serial.print(crc, HEX);
    Serial.print(F(", oldCrc:"));
    Serial.print(old_crc, HEX);
    Serial.print(F(", storedCrc:"));
    Serial.println(readLong(_crcAdr), HEX);
  }
}

void DataStorage::print() {
  Serial.print(F("Stored values. cloudburst:"));
  Serial.print(_cloudburst);
  Serial.print(F(", heavy rain:"));
  Serial.print(_heavyRain);
  Serial.print(F(", light rain:"));
  Serial.println(_lightRain);
}

bool DataStorage::isValid() {
  return readLong(_crcAdr) == calculate_crc();
}

void DataStorage::readValues() {
  _cloudburst = readInt(_cloudBurstAdr); 
  _heavyRain = readInt(_heavyRainAdr);
  _lightRain = readInt(_lightRainAdr);
}

void DataStorage::writeInt(uint16_t adr, uint16_t val) {
  byte low, high;
  low=val&0xFF;
  high=(val>>8)&0xFF;
  EEPROM.update(adr, low); // dauert 3,3ms 
  EEPROM.update(adr+1, high);
} 

uint16_t DataStorage::readInt(uint16_t adr) {
  byte low, high;
  low=EEPROM.read(adr);
  high=EEPROM.read(adr+1);
  return low + ((high << 8)&0xFF00);
}

void DataStorage::writeLong(uint16_t adr, unsigned long val) {
  byte by;
  
  for(int i=0;i< 4;i++) {
    by = (val >> ((3-i)*8)) & 0x000000ff; 
    EEPROM.update(adr+i, by);
  }
}

unsigned long DataStorage::readLong(uint16_t adr) {
  unsigned long val=0;

  for(int i=0;i< 3;i++){
    val += EEPROM.read(adr+i);
    val = val << 8;
  }
  val += EEPROM.read(adr+3);
  return val;
}

unsigned long DataStorage::calculate_crc(void) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;
  int index = 4; // Index starts by 4 because the first 4 bytes include the crc value
  for (; index < EEPROM.length()  ; ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}
