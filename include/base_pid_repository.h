#pragma once

#include "pid_constants.h"

class BasePidRepository {
public:
    BasePidRepository() = default;

    virtual ~BasePidRepository() = default;

    virtual PidConstants_t get() = 0;

    virtual PidConstants_t get(int key) = 0;

    virtual void setup() = 0;

    virtual void save(const PidConstants_t &pid_constants) = 0;

    virtual void save(int key, const PidConstants_t &pid_constants) = 0;
};
