#include "pid_optimizer.h"

void PidOptimizer::saveRollMeasurements(
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
        pid_roll_state_snapshot_array[i + 1] = pid_roll_state_snapshot_array[i];
    }

    pid_roll_state_snapshot_array[0] = PidStateSnapshot(pid_kp, pid_ki, pid_kd, pid_p, pid_i, pid_d, error, previous_error, micros());
}

void PidOptimizer::savePitchMeasurements(
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
        pid_pitch_state_snapshot_array[i + 1] = pid_pitch_state_snapshot_array[i];
    }

    pid_pitch_state_snapshot_array[0] = PidStateSnapshot(pid_kp, pid_ki, pid_kd, pid_p, pid_i, pid_d, error, previous_error, micros());
}

float PidOptimizer::rollScore()
{
    

    return 0;
}

float PidOptimizer::pitchScore()
{
    return 0;
}
