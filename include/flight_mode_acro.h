#ifndef FLIGHT_MODE_ACRO_H
#define FLIGHT_MODE_ACRO_H

#include "base_control_mode.h"

class FlightModeAcro : public BaseControlMode
{
public:
    ControlMode_t type() const override
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
        BaseControlMode::activate(drone, gyro, processor);

        gyro->setModeAcro();
    }
};

inline BaseControlMode* flightModeAcro()
{
    static FlightModeAcro instance;
    return &instance;
}

#endif
