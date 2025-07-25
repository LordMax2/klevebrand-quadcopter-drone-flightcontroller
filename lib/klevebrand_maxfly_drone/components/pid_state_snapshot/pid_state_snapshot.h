#ifndef PID_STATE_SNAPSHOT_H
#define PID_STATE_SNAPSHOT_H

struct PidStateSnapshot {
    PidStateSnapshot() {};
    PidStateSnapshot(
        float pid_kp,
        float pid_ki,
        float pid_kd,
        float pid_p,
        float pid_i,
        float pid_d,
        float error,
        float previous_error,
        long microseconds_timestamp
    ) {
        this->pid_kp = pid_kp;
        this->pid_ki = pid_ki;
        this->pid_kd = pid_kd;
        this->pid_p = pid_p;
        this->pid_i = pid_i;
        this->pid_d = pid_d;
        this->error = error;
        this->previous_error = previous_error;
        this->microseconds_timestamp = microseconds_timestamp; 
    };
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float pid_p;
    float pid_i;
    float pid_d;
    float error;
    float previous_error;
    long microseconds_timestamp; 
};

#endif // PID_STATE_SNAPSHOT_H