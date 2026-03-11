/*
*
*   This part of the drone was possbile because of my genious friend Pio Korinth. 
*   He has explained how a Black Box smiulated annealing works in a way that I could finally wrap my head around.
*
*   Thanks a lot and big credits!
*
*/

#ifndef PID_OPTIMIZER_H
#define PID_OPTIMIZER_H

#include "pid_optimizer_state.h"
#include <math.h>
#include <stdlib.h>

#define TRIAL_DURATION_MILLISECONDS 3000

class PidOptimizer {
private:
    float best_kp;
    float best_ki;
    float best_kd;
    long best_score = 1e10;

public:
    PidOptimizer(float default_kp, float default_ki, float default_kd);

    virtual ~PidOptimizer() = default;

    void run(float current_error, long timestamp_milliseconds);

    float getKp() const { return current_kp; }
    float getKi() const { return current_ki; }
    float getKd() const { return current_kd; }

    virtual float getBestKp() { return best_kp; }
    virtual float getBestKi() { return best_ki; }
    virtual float getBestKd() { return best_kd; }

    virtual void setBestKp(float kp) { best_kp = kp; }
    virtual void setBestKi(float ki) { best_ki = ki; }
    virtual void setBestKd(float kd) { best_kd = kd; }

    float current_kp;
    float current_ki;
    float current_kd;

    virtual void setBestScore(long score) { best_score = score; };
    virtual long getBestScore() { return best_score; };

    PidOptimizerState state;
    unsigned long trial_start_time;
    float error_sum_squared;
    int error_measurement_count;

    void startTrial(long timestamp_milliseconds);

    long score() const;

    virtual void evaluateTrial();

    static float coolingFactor(long timestamp_milliseconds);

    static float fconstrain(float input, float min_value, float max_value);

    static long randomLimited(long min_value, long max_value);
};

#endif // PID_OPTIMIZER_H
