#ifndef PID_YAW_COMPASS_H
#define PID_YAW_COMPASS_H

#include "pid.h"
#include "math.h"

class PidYawCompass : public Pid
{
public:
    PidYawCompass(float kp, float ki, float kd, float pid_max);

    float error(float current, float desired) override;

    static float absoluteCompassError(float current, float desired);
};

#endif // PID_YAW_COMPASS