#include "pid_optimizer.h"

PidOptimizer::PidOptimizer(float initial_kp, float initial_ki, float initial_kd)
{
    current_kp = initial_kp;
    current_ki = initial_ki;
    current_kd = initial_kd;

    best_kp = initial_kp;
    best_ki = initial_ki;
    best_kd = initial_kd;

    best_score = 1e10;

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
        if (millis() - trial_start_time < TRIAL_DURATION_MS)
        {
            error_sum_squared += pow(current_error, 2);
            error_measurement_count++;
        }
        else
        {
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

    float cooling_factor = coolingFactor();
    current_kp += random(-5.0, 5.0) * cooling_factor;
    current_ki += random(-1.0, 1.0) * cooling_factor;
    current_kd += random(-10.0, 10.0) * cooling_factor;

    current_kp = constrain(current_kp, 0.0, 10.0);
    current_ki = constrain(current_ki, 0.0, 2.0);
    current_kd = constrain(current_kd, 0.0, 30.0);

    error_sum_squared = 0;
    error_measurement_count = 0;
    trial_start_time = millis();
    state = MEASURING;
}

long PidOptimizer::score() 
{
    if(error_measurement_count == 0) return 1e10;

    return error_sum_squared / error_measurement_count;
}

void PidOptimizer::evaluateTrial()
{
    long current_score = score();

    if (current_score < best_score)
    {
        best_score = current_score;
        best_kp = current_kp;
        best_ki = current_ki;
        best_kd = current_kd;
    }
    else
    {
        float temperature = 1.0 - coolingFactor();
        float acceptance_probability = exp(-(current_score - best_score) / temperature);
        
        if (random(0.0, 1000.0) / 1000.0 < acceptance_probability)
        {
            best_score = current_score;
            best_kp = current_kp;
            best_ki = current_ki;
            best_kd = current_kd;
        }
    }
}

float PidOptimizer::coolingFactor()
{
    unsigned long time_elapsed = millis();
    float max_duration = 600000; 

    return 1.0 - constrain((float)time_elapsed / max_duration, 0.0, 1.0);
}