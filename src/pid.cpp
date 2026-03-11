#include "pid.h"

void Pid::reset() {
    pid_optimizer = PidOptimizer(_kp, _ki, _kd);
}

void Pid::resetIntegral() {
    pid_i = 0;
    previous_error = 0;
}

float Pid::getKp() const {
    return pid_optimizer.getKp();
}

float Pid::getKi() const {
    return pid_optimizer.getKi();
}

float Pid::getKd() const {
    return pid_optimizer.getKd();
}

void Pid::runOptimizer(const float current, const float desired, const long timestamp_milliseconds) {
    float current_error = error(current, desired);

    pid_optimizer.run(current_error, timestamp_milliseconds);
}

float Pid::error(const float current, const float desired) {
    return desired - current;
}

void Pid::saveError(const float current, const float desired) {
    previous_error = error(current, desired);
}

float Pid::pidP(const float current, const float desired) {
    return pid_optimizer.getKp() * error(current, desired);
}

float Pid::pidD(const float current, const float desired) {
    return pid_optimizer.getKd() * (error(current, desired) - previous_error);
}

float Pid::pid(const float current, const float desired) {
    return fconstrain(pidP(current, desired) + pid_i + pidD(current, desired), -pid_max, pid_max);
}

void Pid::updateIntegral(const float current, const float desired) {
    pid_i = fconstrain(pid_i + (getKi() * error(current, desired)), -pid_max, pid_max);
}

float Pid::fconstrain(const float input, const float min_value, const float max_value) {
    return input < min_value ? min_value : (input > max_value ? max_value : input);
}
