#include "eeprom_pid_repository.h"

void EepromPidRepository::save(PidConstants& pid_constants)
{
    eeprom.writeBlock(0, (uint8_t *)&pid_constants, sizeof(pid_constants));
}

PidConstants EepromPidRepository::get()
{
    PidConstants pid_constants;

    eeprom.readBlock(0, (uint8_t *) &pid_constants, sizeof(pid_constants));

    return pid_constants;
}

void EepromPidRepository::setup()
{
    if (!eeprom.isConnected())
    {
        Serial.println("ERROR: CAN'T FIND EEPROMD...");
    }

    Serial.print("EEPROM CONNECTION STATUS:\t");
    Serial.println(eeprom.isConnected());
}