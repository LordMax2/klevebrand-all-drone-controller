#ifndef FLIGHT_MODE_ACRO_H
#define FLIGHT_MODE_ACRO_H

#include "flight_mode.h"

class FlightModeAcro : public FlightMode {
public:
    FlightMode_t type() const override {
        return acro;
    }

    const char *name() const override {
        return "ACRO";
    }

    int pidConstantsStorageKey() const override {
        return 128;
    }

    PidConstants_t pidConstants() const override {
        return {0.04f, 0.002f, 0.6f, 0.04f, 0.002f, 0.6f, 0.04f, 0.002f, 0.6f};
    }

    void activate(BaseDroneGyro *gyro, BaseHardwareProcessor *processor) const override {
        FlightMode::activate(gyro, processor);

        if (gyro != nullptr) {
            gyro->setModeAcro();
        }
    }
};

inline FlightMode &flightModeAcro() {
    static FlightModeAcro instance;
    return instance;
}

#endif
