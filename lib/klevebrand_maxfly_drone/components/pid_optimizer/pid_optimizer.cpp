#include "pid_optimizer.h"

void PidOptimizer::saveMeasurements(
    float pid_kp,
    float pid_ki,
    float pid_kd,
    float pid_p,
    float pid_i,
    float pid_d,
    float error,
    float previous_error)
{
    for (int i = PID_SNAPSHOT_ARRAY_SIZE - 1; i >= 0; i--)
    {
        pid_state_snapshot_array[i + 1] = pid_state_snapshot_array[i];
    }

    pid_state_snapshot_array[0] = PidStateSnapshot(pid_kp, pid_ki, pid_kd, pid_p, pid_i, pid_d, error, previous_error, micros());
}

float PidOptimizer::score()
{
    float error_score = 0;

    float last_pid_kp = 0;
    float last_pid_ki = 0;
    float last_pid_kd = 0;

    for (int i = 0; i < PID_SNAPSHOT_ARRAY_SIZE - 1; i++)
    {
        // Only calculate the score where the PID constants are the same, otherwise when changed we want to get a new fresh score for the new set of constants
        if(i != 0 && last_pid_kp != pid_state_snapshot_array[i].pid_kp && last_pid_ki != pid_state_snapshot_array[i].pid_ki && last_pid_kd != pid_state_snapshot_array[i].pid_kd) 
        {
            continue;
        }

        error_score += pid_state_snapshot_array[i].error * (pid_state_snapshot_array[i + 1].microseconds_timestamp - pid_state_snapshot_array[i].microseconds_timestamp);

        last_pid_kp = pid_state_snapshot_array[i].pid_kp;
        last_pid_ki = pid_state_snapshot_array[i].pid_ki;
        last_pid_kd = pid_state_snapshot_array[i].pid_kd;
    }

    return error_score;
}

float PidOptimizer::getPAdjustmentValue()
{
    float current_score = score();

    if (current_score < best_score || (current_score != 0 && best_score == 0))
    {
        best_score = current_score;
        best_p_adjustment_value = last_p_adjustment_value;
    }

    last_p_adjustment_value = last_p_adjustment_value + random(-1, 1);

    return last_p_adjustment_value;
}

float PidOptimizer::getIAdjustmentValue()
{
    float current_score = score();

    if (current_score < best_score || (current_score != 0 && best_score == 0))
    {
        best_score = current_score;
        best_i_adjustment_value = last_i_adjustment_value;
    }

    last_i_adjustment_value = last_i_adjustment_value + random(-0.1, 0.1);

    return last_i_adjustment_value;
}

float PidOptimizer::getDAdjustmentValue()
{
    float current_score = score();

    if (current_score < best_score || (current_score != 0 && best_score == 0))
    {
        best_score = current_score;
        best_d_adjustment_value = last_d_adjustment_value;
    }

    last_d_adjustment_value = last_d_adjustment_value + random(-10, 10);

    return last_d_adjustment_value;
}
