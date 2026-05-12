#ifndef BASE_PID_REPOSITORY_H
#define BASE_PID_REPOSITORY_H

#include "pid_constants.h"

class BasePidRepository {
public:
    BasePidRepository() = default;

    virtual ~BasePidRepository() = default;

    virtual PidConstants_t get();

    virtual PidConstants_t get(int key);

    virtual void setup();

    virtual void save(const PidConstants_t &pid_constants);

    virtual void save(int key, const PidConstants_t &pid_constants);
};

#endif
