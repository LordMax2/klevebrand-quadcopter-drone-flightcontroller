#include "eeprom_pid_repository.h"

void EepromPidRepository::save(PidConstants& pid_constants, int address)
{
    eeprom.writeBlock(address, (uint8_t *)&pid_constants, sizeof(pid_constants));
}

PidConstants EepromPidRepository::get(int address)
{
    PidConstants pid_constants;

    eeprom.readBlock(address, (uint8_t *) &pid_constants, sizeof(pid_constants));

    return pid_constants;
}

void EepromPidRepository::setup()
{
    Wire.begin();

    if (!eeprom.isConnected())
    {
        Serial.println("ERROR: CAN'T FIND EEPROMD...");
    }

    Serial.print("EEPROM CONNECTION STATUS:\t");
    Serial.println(eeprom.isConnected());
}