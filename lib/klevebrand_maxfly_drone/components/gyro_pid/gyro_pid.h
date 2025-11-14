#ifndef GYRO_PID_H
#define GYRO_PID_H

#include <Arduino.h>
#include "../pid_optimizer/pid_optimizer.h"

class GyroPid
{
public:
    GyroPid(float kp, float ki, float kd, float pid_max) : pid_max(pid_max), 
                                        pid_roll_optimizer(kp, ki, kd),
                                        pid_pitch_optimizer(kp, ki, kd),
                                        pid_yaw_optimizer(kp, ki, kd) {};
    GyroPid(float kp, float ki, float kd, float yaw_kp, float yaw_ki, float yaw_kd, float pid_max) : pid_max(pid_max),
                                                                                                         pid_roll_optimizer(kp, ki, kd),
                                                                                                         pid_pitch_optimizer(kp, ki, kd),
                                                                                                         pid_yaw_optimizer(yaw_kp, yaw_ki, yaw_kd) {};

    GyroPid(float yaw_kp, float yaw_ki, float yaw_kd, float pitch_kp, float pitch_ki, float pitch_kd, float roll_kp, float roll_ki, float roll_kd, float pid_max) : pid_max(pid_max),
                                                                                                         pid_roll_optimizer(roll_kp, roll_ki, roll_kd),
                                                                                                         pid_pitch_optimizer(pitch_kp, pitch_ki, pitch_kd),
                                                                                                         pid_yaw_optimizer(yaw_kp, yaw_ki, yaw_kd) {};

    void reset();

    void updateIntegral(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    
    void printPid(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    void printPidConstants();

    float getRollKp();
    float getRollKi();
    float getRollKd();

    float getPitchKp();
    float getPitchKi();
    float getPitchKd();

    float getYawKp();
    float getYawKi();
    float getYawKd();

    void runRollOptimizer(float gyro_roll, float roll_desired_angle);
    void runPitchOptimizer(float gyro_pitch, float pitch_desired_angle);
    void runYawOptimizer(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);

    void savePitchError(float gyro_pitch, float pitch_desired_angle);
    void saveRollError(float gyro_roll, float roll_desired_angle);
    void saveYawError(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);

    float yawPid(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    float rollPid(float gyro_roll, float roll_desired_angle);
    float pitchPid(float gyro_pitch, float pitch_desired_angle);

private:
    float pid_max;

    PidOptimizer pid_roll_optimizer;
    PidOptimizer pid_pitch_optimizer;
    PidOptimizer pid_yaw_optimizer;
    
    bool yaw_compass_mode;

    /* Roll PID */
    float rollError(float gyro_roll, float roll_desired_angle);
    float roll_previous_error = 0;
    float rollPidP(float gyro_roll, float roll_desired_angle);
    float roll_pid_i = 0;
    float rollPidD(float gyro_roll, float roll_desired_angle);

    /* Pitch PID */
    float pitchError(float gyro_pitch, float pitch_desired_angle);
    float pitch_previous_error = 0;
    float pitchPidP(float gyro_pitch, float pitch_desired_angle);
    float pitch_pid_i = 0;
    float pitchPidD(float gyro_pitch, float pitch_desired_angle);

    /* Yaw PID */
    float yawError(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    float yaw_previous_error = 0;
    float yawPidP(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    float yaw_pid_i = 0;
    float yawPidD(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
};

#endif // GYRO_PID_H
