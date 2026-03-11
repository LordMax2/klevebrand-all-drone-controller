#include "pid_yaw_compass.h"

PidYawCompass::PidYawCompass(const float kp, const float ki, const float kd, const float pid_max) : Pid(
    kp, ki, kd, pid_max) {
};

float PidYawCompass::error(const float current, const float desired) {
    return absoluteCompassError(current, desired);
}

float PidYawCompass::absoluteCompassError(const float current, const float desired) {
    const float raw_error = current - desired;

    return fmod(raw_error + 540.0f, 360.0f) - 180.0f;
}
