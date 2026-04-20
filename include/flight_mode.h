#ifndef FLIGHT_MODE_H
#define FLIGHT_MODE_H

#include "base_drone_gyro.h"
#include "base_hardware_processor.h"
#include "pid_constants.h"

enum FlightMode_t {
    none = 0,
    auto_level = 1,
    acro = 2
};

class FlightMode {
public:
    FlightMode() = default;

    virtual ~FlightMode() = default;

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

    virtual void activate(BaseDroneGyro *gyro, BaseHardwareProcessor *processor) const {
        if (gyro == nullptr || processor == nullptr) {
            return;
        }

        gyro->reset();
        processor->sleepMilliseconds(1000);
    }
};

inline FlightMode &flightModeNone() {
    static FlightMode instance;
    return instance;
}

#endif // FLIGHT_MODE_H
