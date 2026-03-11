/*
 *
 *   This part of the drone was possbile because of my genious friend Pio Korinth.
 *   He has explained how a Black Box smiulated annealing works in a way that I could finally wrap my head around.
 *
 *   Thanks a lot and big credits!:
 *
 */

#include "pid_optimizer.h"

PidOptimizer::PidOptimizer(const float default_kp, const float default_ki, const float default_kd) : best_kp(0),
    best_ki(0), best_kd(0), trial_start_time(0), error_sum_squared(0), error_measurement_count(0) {
    current_kp = default_kp;
    current_ki = default_ki;
    current_kd = default_kd;

    PidOptimizer::setBestKp(default_kp);
    PidOptimizer::setBestKi(default_ki);
    PidOptimizer::setBestKd(default_kd);

    state = IDLE;
}

void PidOptimizer::run(float current_error, long timestamp_milliseconds) {
    switch (state) {
        case IDLE:
            startTrial(timestamp_milliseconds);
            break;

        case MEASURING:
            if (timestamp_milliseconds - trial_start_time < TRIAL_DURATION_MILLISECONDS) {
                error_sum_squared += pow(fabs(current_error), 2);
                error_measurement_count++;
            } else {
                // If we don't get enough readings, restart the trial
                if (error_measurement_count < ((TRIAL_DURATION_MILLISECONDS / 1000) * 200) * 0.9)
                // TODO: Replace hardcoded 200 with the Flight Controller hz frequency, and the acceptance percentage deviation.
                {
                    startTrial(timestamp_milliseconds);

                    return;
                }

                state = DECIDING;
            }
            break;

        case DECIDING:
            evaluateTrial();
            state = IDLE;
            break;
    }
}

void PidOptimizer::startTrial(long timestamp_milliseconds) {
    current_kp = getBestKp();
    current_ki = getBestKi();
    current_kd = getBestKd();

    current_kp += randomLimited(-5, 5) / 10.0f;
    current_ki += randomLimited(-3, 3) / 10000.0f;
    current_kd += randomLimited(-10, 10) / 1.0f;

    current_kp = fconstrain(current_kp, 0.1f, 10.0f);
    current_ki = fconstrain(current_ki, 0.00001f, 0.005f);
    current_kd = fconstrain(current_kd, 0.1f, 100.0f);

    error_sum_squared = 0;
    error_measurement_count = 0;
    trial_start_time = timestamp_milliseconds;
    state = MEASURING;
}

long PidOptimizer::randomLimited(const long min_value, const long max_value) {
    if (min_value >= max_value) {
        return min_value;
    }
    const long diff = max_value - min_value;

    if (max_value == 0) {
        return 0;
    }
    return (rand() % diff) + min_value;
}

long PidOptimizer::score() const {
    if (error_measurement_count == 0)
        return 1e10;

    return error_sum_squared / error_measurement_count;
}

void PidOptimizer::evaluateTrial() {
    const long current_score = score();

    if (current_score == 0)
        return;

    if (current_score < getBestScore()) {
        setBestScore(current_score);

        setBestKp(current_kp);
        setBestKi(current_ki);
        setBestKd(current_kd);
    } else {
        constexpr float temperature = 0.1f; // Diable temperature, dangerous for real flight, cost new propellers hehe

        if (temperature == 0.0f)
            return;

        float acceptance_probability = exp(-(current_score - getBestScore()) / temperature);

        if (randomLimited(0.0, 1000.0) / 1000.0 < acceptance_probability) {
            setBestScore(current_score);

            setBestKp(current_kp);
            setBestKi(current_ki);
            setBestKd(current_kd);
        }
    }
}

float PidOptimizer::coolingFactor(const long timestamp_milliseconds) {
    const unsigned long time_elapsed = timestamp_milliseconds;
    constexpr float cooling_duration = 600000;

    return 1.0 - fconstrain((float) time_elapsed / cooling_duration, 0.0, 1.0);
}

float PidOptimizer::fconstrain(const float input, const float min_value, const float max_value) {
    return input < min_value ? min_value : (input > max_value ? max_value : input);
}
