#ifndef EEPROM_PID_REPOSITORY_H
#define EEPROM_PID_REPOSITORY_H

#include <I2C_eeprom.h>
#include <Wire.h>
#include "./pid_constants.h"

class EepromPidRepository
{
public:
    EepromPidRepository() : eeprom(0x50, I2C_DEVICESIZE_24LC512) {}

    void setup();
    void save(PidConstants& pid_constants, int address);
    PidConstants get(int address);

private:
    I2C_eeprom eeprom;
};

#endif // EEPROM_PID_REPOSITORY_H