#include "eeprom_pid_repository.h"

void EepromPidRepository::save(int key, PidConstants_t& pid_constants)
{
    eeprom.writeBlock(key, (uint8_t *)&pid_constants, sizeof(pid_constants));
}

PidConstants_t EepromPidRepository::get(int key)
{
    PidConstants_t pid_constants;

    eeprom.readBlock(key, (uint8_t *) &pid_constants, sizeof(pid_constants));

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