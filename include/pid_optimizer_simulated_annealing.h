#ifndef PID_OPTIMIZER_SIMULATED_ANNEALING_H
#define PID_OPTIMIZER_SIMULATED_ANNEALING_H

#include "pid_optimizer.h"

class PidOptimizerSimulatedAnnealing : public PidOptimizer
{
private:
    long best_score = 1e10;

public:
    PidOptimizerSimulatedAnnealing(float default_kp, float default_ki, float default_kd);

    void evaluateTrial() override;
};

#endif // PID_OPTIMIZER_SIMULATED_ANNEALING_H