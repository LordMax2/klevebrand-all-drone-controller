#include "pid_yaw_compass.h"

PidYawCompass::PidYawCompass(float kp, float ki, float kd, float pid_max) : Pid(kp, ki, kd, pid_max) {};

float PidYawCompass::error(float current, float desired)
{
    return absoluteCompassError(current, desired);
}

float PidYawCompass::absoluteCompassError(float current, float desired)
{
    float raw_error = current - desired;

    return fmod(raw_error + 540.0f, 360.0f) - 180.0f;
}