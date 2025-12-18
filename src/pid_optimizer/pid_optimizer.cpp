/*
 *
 *   This part of the drone was possbile because of my genious friend Pio Korinth.
 *   He has explained how a Black Box smiulated annealing works in a way that I could finally wrap my head around.
 *
 *   Thanks a lot and big credits!:
 *
 */

#include "pid_optimizer.h"

PidOptimizer::PidOptimizer(float initial_kp, float initial_ki, float initial_kd)
{
    current_kp = initial_kp;
    current_ki = initial_ki;
    current_kd = initial_kd;

    best_kp = initial_kp;
    best_ki = initial_ki;
    best_kd = initial_kd;

    previous_score = 1e10;

    state = IDLE;
}

void PidOptimizer::run(float current_error)
{
    switch (state)
    {
    case IDLE:
        startTrial();
        break;

    case MEASURING:
        if (millis() - trial_start_time < TRIAL_DURATION_MILLISECONDS)
        {
            error_sum_squared += pow(current_error, 2);
            error_measurement_count++;
        }
        else
        {
            // If we dont get enough readings, restart the trial
            if (error_measurement_count < ((TRIAL_DURATION_MILLISECONDS / 1000) * 200) * 0.9) // TODO: Replace hardcoded 200 with the Flight Controller hz frequency, and the acceptance percentage deviation.
            {
                startTrial();

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

void PidOptimizer::startTrial()
{
    current_kp = best_kp;
    current_ki = best_ki;
    current_kd = best_kd;

    current_kp += random(-5, 5) / 10000.0f;
    current_ki += random(-3, 3) / 100000.0f;
    current_kd += random(-10, 10) / 10000.0f;

    current_kp = constrain(current_kp, 0.1f, 1.0f);
    current_ki = constrain(current_ki, 0.0001f, 0.05f);
    current_kd = constrain(current_kd, 0.1f, 20.0f);

    error_sum_squared = 0;
    error_measurement_count = 0;
    trial_start_time = millis();
    state = MEASURING;
}

long PidOptimizer::score()
{
    if (error_measurement_count == 0)
        return 1e10;

    return error_sum_squared / error_measurement_count;
}

void PidOptimizer::evaluateTrial()
{
    long current_score = score();

    float factor = 0.9f;

    // If current score is "worse" than the previous score, reverse the direction of the values
    if (current_score > previous_score)
    {
        current_kp = (current_kp - best_kp) * -1 + best_kp;
        current_ki = (current_ki - best_ki) * -1 + best_ki;
        current_kd = (current_kd - best_kd) * -1 + best_kd;
    }

    best_kp = best_kp * factor + current_kp * (1.0f - factor);
    best_ki = best_ki * factor + current_ki * (1.0f - factor);
    best_kd = best_kd * factor + current_kd * (1.0f - factor);

    previous_score = current_score;
}

float PidOptimizer::coolingFactor()
{
    unsigned long time_elapsed = millis();
    float cooling_duration = 600000;

    return 1.0 - constrain((float)time_elapsed / cooling_duration, 0.0, 1.0);
}
