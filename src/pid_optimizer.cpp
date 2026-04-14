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
    PidOptimizer::setPreviousScore(1e10);

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

    float noise = 0.1f;
    current_kp += step_p + (randomLimited(-5, 5) / 10000.0f) * noise;
    current_ki += step_i + (randomLimited(-3, 3) / 100000.0f) * noise;
    current_kd += step_d + (randomLimited(-10, 10) / 10000.0f) * noise;

    current_kp = fconstrain(current_kp, 0.1f, 10.0f);
    current_ki = fconstrain(current_ki, 0.0001f, 0.05f);
    current_kd = fconstrain(current_kd, 0.1f, 20.0f);

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

    const float diff_p = current_kp - getBestKp();
    const float diff_i = current_ki - getBestKi();
    const float diff_d = current_kd - getBestKd();

    if (current_score < getPreviousScore()) {
        constexpr float factor = 0.95f;

        step_p = (1.0f - learning_rate) * step_p + learning_rate * diff_p;
        step_i = (1.0f - learning_rate) * step_i + learning_rate * diff_i;
        step_d = (1.0f - learning_rate) * step_d + learning_rate * diff_d;

        setBestKp(getBestKp() * factor + current_kp * (1.0f - factor));
        setBestKi(getBestKi() * factor + current_ki * (1.0f - factor));
        setBestKd(getBestKd() * factor + current_kd * (1.0f - factor));

        setPreviousScore(current_score);
    } else {
        step_p *= -0.5f;
        step_i *= -0.5f;
        step_d *= -0.5f;
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
