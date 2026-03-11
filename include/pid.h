#ifndef THROTTLE_PID_H
#define THROTTLE_PID_H

#include "pid_optimizer.h"
#include <math.h>

class Pid {
public:
    Pid(const float kp, const float ki, const float kd, const float pid_max) : pid_max(pid_max),
                                                       pid_optimizer(kp, ki, kd) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    };
    virtual ~Pid() = default;

    void reset();

    void updateIntegral(float current, float desired);

    void resetIntegral();

    float getKp() const;

    float getKi() const;

    float getKd() const;

    void runOptimizer(float current, float desired, long timestamp_milliseconds);

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

    static float fconstrain(float input, float min_value, float max_value);
};

#endif // THROTTLE_PID_H
