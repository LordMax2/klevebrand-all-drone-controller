#ifndef FLIGHT_MODE_AUTO_LEVEL
#define FLIGHT_MODE_AUTO_LEVEL

#include "flight_mode.h"

class FlightModeAutoLevel : public FlightMode {
public:
    FlightMode_t type() const override {
        return auto_level;
    }

    const char *name() const override {
        return "AUTOLEVEL";
    }

    int pidConstantsStorageKey() const override {
        return 256;
    }

    bool yawCompassMode() const override {
        return true;
    }

    PidConstants_t pidConstants() const override {
        return {0.125f, 0.001f, 2.5f, 0.125f, 0.001f, 0.25f, 0.05f, 0.0005f, 0.2f};
    }

    void activate(BaseDroneGyro *gyro, BaseHardwareProcessor *processor) const override {
        FlightMode::activate(gyro, processor);

        if (gyro != nullptr) {
            gyro->setModeEuler();
        }
    }
};

inline const FlightMode &flightModeAutoLevel() {
    static const FlightModeAutoLevel instance;
    return instance;
}

#endif