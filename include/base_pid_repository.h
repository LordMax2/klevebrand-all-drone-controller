#ifndef TEMPLATE_PID_REPOSITORY_H
#define TEMPLATE_PID_REPOSITORY_H

#include "pid_constants.h"

class BasePidRepository
{
public:
    virtual PidConstants_t get() { return PidConstants_t(0, 0, 0, 0, 0, 0, 0, 0, 0); }
    virtual PidConstants_t get(int key) { return PidConstants_t(0, 0, 0, 0, 0, 0, 0, 0, 0); }
    virtual void setup() {}
    virtual void save(PidConstants_t &pid_constants) {}
    virtual void save(int key, PidConstants_t &pid_constants) {}
};

#endif // TEMPLATE_PID_REPOSITORY_H