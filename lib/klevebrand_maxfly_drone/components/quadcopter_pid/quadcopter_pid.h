#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "../pid_optimizer/pid_optimizer.h"

#define PID_THROTTLE_THRESHOLD 1100
#define PID_MAX 400

#define THROTTLE_MINIMUM 1000
#define THROTTLE_MAXIMUM 2000

class Pid
{
public:
    Pid(float kp, float ki, float kd) : pid_roll_optimizer(kp, ki, kd),
                                        pid_pitch_optimizer(kp, ki, kd),
                                        pid_yaw_optimizer(kp, ki, kd) {};
    Pid(float kp, float ki, float kd, float yaw_kp, float yaw_ki, float yaw_kd) : pid_roll_optimizer(kp, ki, kd),
                                                                                                         pid_pitch_optimizer(kp, ki, kd),
                                                                                                         pid_yaw_optimizer(yaw_kp, yaw_ki, yaw_kd) {};
    void reset();
    void updateIntegral(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    void printPid(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    void printPidConstants();
    void runRollOptimizer(float gyro_roll, float roll_desired_angle);
    void runPitchOptimizer(float gyro_pitch, float pitch_desired_angle);
    void runYawOptimizer(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);

    float pidThrottleLF(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
    {
        if (throttle < PID_THROTTLE_THRESHOLD)
            return THROTTLE_MINIMUM;

        return constrain(throttle + rollPid(gyro_roll, roll_desired_angle) - pitchPid(gyro_pitch, pitch_desired_angle) + yawPid(gyro_yaw, yaw_desired_angle, yaw_compass_mode), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    float pidThrottleLB(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
    {
        if (throttle < PID_THROTTLE_THRESHOLD)
            return THROTTLE_MINIMUM;

        return constrain(throttle + rollPid(gyro_roll, roll_desired_angle) + pitchPid(gyro_pitch, pitch_desired_angle) - yawPid(gyro_yaw, yaw_desired_angle, yaw_compass_mode), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    float pidThrottleRF(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
    {
        if (throttle < PID_THROTTLE_THRESHOLD)
            return THROTTLE_MINIMUM;

        return constrain(throttle - rollPid(gyro_roll, roll_desired_angle) - pitchPid(gyro_pitch, pitch_desired_angle) - yawPid(gyro_yaw, yaw_desired_angle, yaw_compass_mode), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    float pidThrottleRB(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
    {
        if (throttle < PID_THROTTLE_THRESHOLD)
            return THROTTLE_MINIMUM;

        return constrain(throttle - rollPid(gyro_roll, roll_desired_angle) + pitchPid(gyro_pitch, pitch_desired_angle) + yawPid(gyro_yaw, yaw_desired_angle, yaw_compass_mode), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    void savePitchError(float gyro_pitch, float pitch_desired_angle);
    void saveRollError(float gyro_roll, float roll_desired_angle);
    void saveYawError(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);

private:
    long previous_timer;
    PidOptimizer pid_roll_optimizer;
    PidOptimizer pid_pitch_optimizer;
    PidOptimizer pid_yaw_optimizer;
    bool yaw_compass_mode;

    /* Roll PID */
    float rollPid(float gyro_roll, float roll_desired_angle);
    float rollError(float gyro_roll, float roll_desired_angle);
    float roll_previous_error = 0;
    float rollPidP(float gyro_roll, float roll_desired_angle);
    float roll_pid_i = 0;
    float rollPidD(float gyro_roll, float roll_desired_angle);

    /* Pitch PID */
    float pitchPid(float gyro_pitch, float pitch_desired_angle);
    float pitchError(float gyro_pitch, float pitch_desired_angle);
    float pitch_previous_error = 0;
    float pitchPidP(float gyro_pitch, float pitch_desired_angle);
    float pitch_pid_i = 0;
    float pitchPidD(float gyro_pitch, float pitch_desired_angle);

    /* Yaw PID */
    float yawPid(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    float yawError(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    float yaw_previous_error = 0;
    float yawPidP(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    float yaw_pid_i = 0;
    float yawPidD(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
};

#endif
