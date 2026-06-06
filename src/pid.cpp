#include "pid.h"

void Pid::reset() {
    _pid_optimizer = PidOptimizer(_kp, _ki, _kd, _feedback_loop_hz);

    resetIntegral();
}

void Pid::resetIntegral() {
    _pid_i = 0;
    _previous_error = 0;
}

float Pid::getKp() const {
    return _pid_optimizer.getKp();
}

float Pid::getKi() const {
    return _pid_optimizer.getKi();
}

float Pid::getKd() const {
    return _pid_optimizer.getKd();
}

void Pid::runOptimizer(const float current, const float desired, const long timestamp_milliseconds) {
    const float current_error = error(current, desired);

    _pid_optimizer.run(current_error, timestamp_milliseconds);
}

float Pid::error(const float current, const float desired) {
    return desired - current;
}

void Pid::saveError(const float current, const float desired) {
    _previous_error = error(current, desired);
    _previous_current = current;
}

float Pid::pidP(const float current, const float desired) {
    return _pid_optimizer.getKp() * error(current, desired);
}

float Pid::pidD(const float current, const float desired, const float delta_time_seconds) {
    if (delta_time_seconds <= 0.0f) {
        return 0.0f;
    }

    return _pid_optimizer.getKd() * ((_previous_current - current) / delta_time_seconds);
}

float Pid::pid(const float current, const float desired, const float delta_time_seconds) {
    return fconstrain(pidP(current, desired) + _pid_i + pidD(current, desired, delta_time_seconds), -_pid_max, _pid_max);
}

void Pid::updateIntegral(const float current, const float desired, const float delta_time_seconds) {
    if (delta_time_seconds <= 0.0f) {
        return;
    }

    _pid_i = fconstrain(_pid_i + (getKi() * error(current, desired) * delta_time_seconds), -_pid_max, _pid_max);
}

float Pid::fconstrain(const float input, const float min_value, const float max_value) {
    return input < min_value ? min_value : input > max_value ? max_value : input;
}
