#include "pid_yaw_compass.h"
#include "math.h"

PidYawCompass::PidYawCompass(const float kp, const float ki, const float kd, const float pid_max, const int feedback_loop_hz) : Pid(
    kp, ki, kd, pid_max, feedback_loop_hz) {
};

float PidYawCompass::error(const float current, const float desired) {
    return absoluteCompassError(current, desired);
}

float PidYawCompass::pidD(const float current, const float desired, const float delta_time_seconds) {
    if (delta_time_seconds <= 0.0f) {
        return 0.0f;
    }

    return getKd() * ((error(current, desired) - previousError()) / delta_time_seconds);
}

float PidYawCompass::absoluteCompassError(const float current, const float desired) {
    const float raw_error = current - desired;

    return fmod(raw_error + 540.0f, 360.0f) - 180.0f;
}
