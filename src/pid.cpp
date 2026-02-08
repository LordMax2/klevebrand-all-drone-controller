#include "pid.h"

void Pid::reset()
{
    pid_optimizer = PidOptimizer(_kp, _ki, _kd);
}

void Pid::printConstants()
{
    Serial.print("Throttle  PID Constants - Kp: ");
    Serial.print(_kp);
    Serial.print(", Ki: ");
    Serial.print(_ki);
    Serial.print(", Kd: ");
    Serial.println(_kd);
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

void Pid::runOptimizer(float current, float desired)
{
    float current_error = error(current, desired);

    pid_optimizer.run(current_error);
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
    return constrain(pidP(current, desired) + pid_i + pidD(current, desired), -pid_max, pid_max);
}

void Pid::printPid(float current_, float desired)
{
    Serial.print("Throttle  PID: ");
    Serial.println(pid(current_, desired));
}

void Pid::updateIntegral(float current, float desired)
{
    pid_i = constrain(pid_i + (getKi() * error(current, desired)), -pid_max, pid_max);
}