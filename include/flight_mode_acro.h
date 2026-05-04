#ifndef FLIGHT_MODE_ACRO_H
#define FLIGHT_MODE_ACRO_H

#include "base_flight_mode.h"

class FlightModeAcro : public BaseFlightMode
{
public:
    FlightMode_t type() const override
    {
        return acro;
    }

    const char* name() const override
    {
        return "ACRO";
    }

    int pidConstantsStorageKey() const override
    {
        return 128;
    }

    PidConstants_t pidConstants() const override
    {
        return {0.04f, 0.002f, 0.6f, 0.04f, 0.002f, 0.6f, 0.04f, 0.002f, 0.6f};
    }

    void activate(BaseDrone* drone, BaseDroneGyro* gyro, BaseHardwareProcessor* processor) const override
    {
        BaseFlightMode::activate(drone, gyro, processor);

        gyro->setModeAcro();
    }
};

inline BaseFlightMode* flightModeAcro()
{
    static FlightModeAcro instance;
    return &instance;
}

#endif
