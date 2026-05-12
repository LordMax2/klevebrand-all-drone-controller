#ifndef FLIGHT_MODE_AUTO_LEVEL_H
#define FLIGHT_MODE_AUTO_LEVEL_H

#include "base_drone.h"
#include "base_control_mode.h"

class FlightModeAutoLevel : public BaseControlMode
{
public:
    ControlMode_t type() const override
    {
        return auto_level;
    }

    const char* name() const override
    {
        return "AUTOLEVEL";
    }

    int pidConstantsStorageKey() const override
    {
        return 256;
    }

    bool yawCompassMode() const override
    {
        return true;
    }

    PidConstants_t pidConstants() const override
    {
        return {0.125f, 0.001f, 2.5f, 0.125f, 0.001f, 0.25f, 0.05f, 0.0005f, 0.2f};
    }

    void activate(BaseDrone* drone, BaseDroneGyro* gyro, BaseHardwareProcessor* processor) const override
    {
        BaseControlMode::activate(drone, gyro, processor);

        gyro->setModeEuler();

        drone->setDesiredYawAngle(gyro->yaw());
    }
};

inline BaseControlMode* flightModeAutoLevel()
{
    static FlightModeAutoLevel instance;
    return &instance;
}

#endif
