#include "pid_optimizer_simulated_annealing.h"

PidOptimizerSimulatedAnnealing::PidOptimizerSimulatedAnnealing(float default_kp, float default_ki, float default_kd)
    : PidOptimizer(default_kp, default_ki, default_kd) {};

void PidOptimizerSimulatedAnnealing::evaluateTrial()
{
    long current_score = score();

    if (current_score == 0)
        return;

    if (current_score < best_score)
    {
        best_score = current_score;

        best_kp = current_kp;
        best_ki = current_ki;
        best_kd = current_kd;
    }
    else
    {
        float temperature = 1.0f; // Diable temprature, dangerous for real flight, cost new propellers hehe

        if (temperature == 0.0f)
            return;

        float acceptance_probability = exp(-(current_score - best_score) / temperature);

        if (randomLimited(0.0, 1000.0) / 1000.0 < acceptance_probability)
        {
            best_score = current_score;

            best_kp = current_kp;
            best_ki = current_ki;
            best_kd = current_kd;
        }
    }
}