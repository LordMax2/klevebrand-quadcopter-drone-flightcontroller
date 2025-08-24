/*
*
*   This part of the drone was possbile because of my genious friend Pio Korinth. 
*   He has explained how a Black Box smiulated annealing works in a way that I could finally wrap my head around.
*
*   Thanks a lot and big credits!: 
*
*/

#ifndef PID_OPTIMIZER_H
#define PID_OPTIMIZER_H

#include <Arduino.h>

#define TRIAL_DURATION_MILLISECONDS 3000 

enum PidOptimizerState {
    IDLE,
    MEASURING,
    DECIDING
};

class PidOptimizer {
public:
    PidOptimizer(float default_kp, float default_ki, float default_kd);

    void run(float current_error);

    float getKp() { return current_kp; }
    float getKi() { return current_ki; }
    float getKd() { return current_kd; }

private:
    float current_kp;
    float current_ki;
    float current_kd;

    float best_kp;
    float best_ki;
    float best_kd;
    
    float best_score;

    PidOptimizerState state;
    unsigned long trial_start_time;
    float error_sum_squared;
    int error_measurement_count;

    void startTrial();
    long score();
    void evaluateTrial();
    float coolingFactor();
};

#endif // PID_OPTIMIZER_H
