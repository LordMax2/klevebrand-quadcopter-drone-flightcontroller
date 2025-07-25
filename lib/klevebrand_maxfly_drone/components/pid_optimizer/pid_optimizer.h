#ifndef PID_OPTIMIZER_H
#define PID_OPTIMIZER_H

#include <Arduino.h>
#include "../pid_state_snapshot/pid_state_snapshot.h"

#define PID_SNAPSHOT_ARRAY_SIZE 50 

class PidOptimizer {
public:
 void saveRollMeasurements(float pid_kp, float pid_ki, float pid_kd, float pid_p, float pid_i, float pid_d, float error, float previous_error);
 void savePitchMeasurements(float pid_kp, float pid_ki, float pid_kd, float pid_p, float pid_i, float pid_d, float error, float previous_error);
 float rollScore();
 float pitchScore();
private:
    PidStateSnapshot pid_roll_state_snapshot_array[PID_SNAPSHOT_ARRAY_SIZE];
    PidStateSnapshot pid_pitch_state_snapshot_array[PID_SNAPSHOT_ARRAY_SIZE];
};

#endif // PID_OPTIMIZER_H
