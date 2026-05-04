#ifndef FLIGHT_MODE_H
#define FLIGHT_MODE_H

#include "base_drone_gyro.h"
#include "base_hardware_processor.h"
#include "pid_constants.h"

class BaseDrone;

enum FlightMode_t {
    none = 0,
    auto_level = 1,
    acro = 2
};

class BaseFlightMode {
public:
    BaseFlightMode() = default;

    virtual ~BaseFlightMode() = default;

    virtual FlightMode_t type() const {
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

inline BaseFlightMode *flightModeNone() {
    static BaseFlightMode instance;
    return &instance;
}

#endif // FLIGHT_MODE_H
