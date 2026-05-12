#ifndef BASE_CONTROL_MODE_H
#define BASE_CONTROL_MODE_H

#include "base_drone_gyro.h"
#include "base_hardware_processor.h"
#include "pid_constants.h"

class BaseDrone;

enum ControlMode_t {
    none = 0,
    auto_level = 1,
    acro = 2
};

class BaseControlMode {
public:
    BaseControlMode() = default;

    virtual ~BaseControlMode() = default;

    virtual ControlMode_t type() const {
        return none;
    }

    virtual const char *name() const {
        return "NONE";
    }

    virtual int pidConstantsStorageKey() const {
        return 0;
    }

    virtual bool yawCompassMode() const {
        return false;
    }

    virtual PidConstants_t pidConstants() const {
        return {};
    }

    virtual void activate(BaseDrone* drone, BaseDroneGyro *gyro, BaseHardwareProcessor *processor) const {
        gyro->reset();
        processor->sleepMilliseconds(1000);
    }
};

inline BaseControlMode *controlModeNone() {
    static BaseControlMode instance;
    return &instance;
}

#endif
