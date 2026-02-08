#ifndef PID_YAW_COMPASS_H
#define PID_YAW_COMPASS_H

#include "pid.h"

class PidYawCompass : public Pid
{
public:
    PidYawCompass(float kp, float ki, float kd, float pid_max) : Pid(kp, ki, kd, pid_max) {};

    float error(float current, float desired) override
    {
        auto error = fmod((current + 180), 360) - fmod((desired + 180), 360);
        float absolute_error = min(abs(error), 360 - abs(error));

        if (error < 0)
        {
            return -1 * absolute_error;
        }

        return absolute_error;
    }
};

#endif // PID_YAW_COMPASS