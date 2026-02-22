#ifndef PID_YAW_COMPASS_H
#define PID_YAW_COMPASS_H

#include "pid.h"
#include "math.h"

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
        float raw_error = current - desired;

        return fmod(raw_error + 540.0f, 360.0f) - 180.0f;
    }

};

#endif // PID_YAW_COMPASS