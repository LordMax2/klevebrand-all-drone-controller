#ifndef THROTTLE_PID_H
#define THROTTLE_PID_H

#include <Arduino.h>
#include "pid_optimizer.h"

class Pid
{
public:
    Pid(float kp, float ki, float kd, float pid_max) : pid_max(pid_max),
                                                                       pid_optimizer(kp, ki, kd)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    };

    void reset();

    void updateIntegral(float current, float desired);

    void printPid(float current, float desired);
    void printConstants();

    float getKp();
    float getKi();
    float getKd();

    void runOptimizer(float current, float desired);

    void saveError(float current, float desired);

    float pid(float current, float desired);

    float pid_max;
    PidOptimizer pid_optimizer;

    virtual float error(float current, float desired);
    float previous_error = 0;

    float pidP(float current, float desired);
    float pid_i = 0;
    float pidD(float current, float desired);

    float _kp;
    float _ki;
    float _kd;
};

#endif // THROTTLE_PID_H