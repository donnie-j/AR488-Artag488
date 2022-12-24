#ifndef AR488_EEPROM_H
#define AR488_EEPROM_H

#include "AR488_Config.h"

#define EESIZE 512
#define EESTART 2    // EEPROM start of data - min 4 for CRC32, min 2 for CRC16
#define UPCASE true

const uint16_t eesize = EESIZE;

void epErase();
void epWriteData(uint8_t cfgdata[], uint16_t cfgsize);
bool epReadData(uint8_t cfgdata[], uint16_t cfgsize);
void epViewData(Stream& outputStream);
bool isEepromClear();

#endif // AR488_EEPROM_H
