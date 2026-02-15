#include "pid.hpp"

void Pid::reset()
{
    pid_optimizer = PidOptimizer(_kp, _ki, _kd);
}

float Pid::getKp()
{
    return pid_optimizer.getKp();
}

float Pid::getKi()
{
    return pid_optimizer.getKi();
}

float Pid::getKd()
{
    return pid_optimizer.getKd();
}

void Pid::runOptimizer(float current, float desired, long timstamp_milliseconds)
{
    float current_error = error(current, desired);

    pid_optimizer.run(current_error, timstamp_milliseconds);
}

float Pid::error(float current, float desired)
{
    return desired - current;
}

void Pid::saveError(float current, float desired)
{
    previous_error = error(current, desired);
}

float Pid::pidP(float current, float desired)
{
    return pid_optimizer.getKp() * error(current, desired);
}

float Pid::pidD(float current, float desired)
{
    return pid_optimizer.getKd() * (error(current, desired) - previous_error);
}

float Pid::pid(float current, float desired)
{
    return fconstrain(pidP(current, desired) + pid_i + pidD(current, desired), -pid_max, pid_max);
}

void Pid::updateIntegral(float current, float desired)
{
    pid_i = fconstrain(pid_i + (getKi() * error(current, desired)), -pid_max, pid_max);
}

float Pid::fconstrain(float input, float min_value, float max_value)
{
    return input < min_value ? min_value : (input > max_value ? max_value : input);
}