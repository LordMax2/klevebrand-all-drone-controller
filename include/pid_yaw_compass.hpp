#ifndef PID_YAW_COMPASS_H
#define PID_YAW_COMPASS_H

#include "pid.hpp"

class PidYawCompass : public Pid
{
public:
    PidYawCompass(float kp, float ki, float kd, float pid_max) : Pid(kp, ki, kd, pid_max) {};

    float error(float current, float desired) override
    {
        return absoluteCompassError(current, desired);
    }

    static float absoluteCompassError(float current, float desired)
    {
        auto error = fmod((current + 180), 360) - fmod((desired + 180), 360);
        float absolute_error = fmin(fabs(error), 360 - fabs(error));

        if (error < 0)
        {
            return -1 * absolute_error;
        }

        return absolute_error;
    }
};

#endif // PID_YAW_COMPASS