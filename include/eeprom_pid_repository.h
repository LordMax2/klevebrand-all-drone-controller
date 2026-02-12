#ifndef EEPROM_PID_REPOSITORY_H
#define EEPROM_PID_REPOSITORY_H

#include <I2C_eeprom.h>
#include <Wire.h>
#include "pid_constants.h"
#include "template_pid_repository.h"

class EepromPidRepository : public TemplatePidRepository<I2C_eeprom>
{
public:
    EepromPidRepository() : TemplatePidRepository(&eeprom), eeprom(0x50, I2C_DEVICESIZE_24LC512) {}

    void setup();
    void save(int key, PidConstants_t& pid_constants) override;
    PidConstants_t get(int key) override;

private:
    I2C_eeprom eeprom;
};

#endif // EEPROM_PID_REPOSITORY_H