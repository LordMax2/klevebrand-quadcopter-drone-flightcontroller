#ifndef PID_OPTIMIZER_H
#define PID_OPTIMIZER_H

#include <Arduino.h>
#include "../pid_state_snapshot/pid_state_snapshot.h"

#define PID_SNAPSHOT_ARRAY_SIZE 50 

class PidOptimizer {
public:
 void saveMeasurements(float pid_kp, float pid_ki, float pid_kd, float pid_p, float pid_i, float pid_d, float error, float previous_error);
 float score();
 float getPAdjustmentValue();
 float getIAdjustmentValue();
 float getDAdjustmentValue();
private:
    PidStateSnapshot pid_state_snapshot_array[PID_SNAPSHOT_ARRAY_SIZE];

    float best_score = 0;

    float last_p_adjustment_value = 0;
    float best_p_adjustment_value = 0;

    float last_i_adjustment_value = 0;
    float best_i_adjustment_value = 0;

    float last_d_adjustment_value = 0;
    float best_d_adjustment_value = 0;
};

#endif // PID_OPTIMIZER_H
