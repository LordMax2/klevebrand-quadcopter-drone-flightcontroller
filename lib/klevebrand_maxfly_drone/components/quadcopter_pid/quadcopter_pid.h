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
    Pid(float kp, float ki, float kd) : pid_roll_optimizer(kp, ki, kd), pid_pitch_optimizer(kp, ki, kd), pid_yaw_optimizer(kp, ki, kd) { };
    void reset();
    void updateIntegral(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle);
    void printPid(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle);
    void printPidConstants();
    void runRollOptimizer(float gyro_roll, float roll_desired_angle);
    void runPitchOptimizer(float gyro_pitch, float pitch_desired_angle);

    float pidThrottleLF(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle)
    {
        if (throttle < PID_THROTTLE_THRESHOLD)
            return THROTTLE_MINIMUM;

        return constrain(throttle + rollPid(gyro_roll, roll_desired_angle) - pitchPid(gyro_pitch, pitch_desired_angle) + yawPid(gyro_yaw, yaw_desired_angle), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    float pidThrottleLB(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle)
    {
        if (throttle < PID_THROTTLE_THRESHOLD)
            return THROTTLE_MINIMUM;

        return constrain(throttle + rollPid(gyro_roll, roll_desired_angle) + pitchPid(gyro_pitch, pitch_desired_angle) - yawPid(gyro_yaw, yaw_desired_angle), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    float pidThrottleRF(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle)
    {
        if (throttle < PID_THROTTLE_THRESHOLD)
            return THROTTLE_MINIMUM;

        return constrain(throttle - rollPid(gyro_roll, roll_desired_angle) - pitchPid(gyro_pitch, pitch_desired_angle) - yawPid(gyro_yaw, yaw_desired_angle), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    float pidThrottleRB(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle)
    {
        if (throttle < PID_THROTTLE_THRESHOLD)
            return THROTTLE_MINIMUM;

        return constrain(throttle - rollPid(gyro_roll, roll_desired_angle) + pitchPid(gyro_pitch, pitch_desired_angle) + yawPid(gyro_yaw, yaw_desired_angle), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    void savePitchError(float gyro_pitch, float pitch_desired_angle);
    void saveRollError(float gyro_roll, float roll_desired_angle);
    void saveYawError(float gyro_yaw, float yaw_desired_angle);

    // Working PID contants: 
    /*
        kp = 1.25
        ki = 0.01
        kd = 25
    */

private:
    long previous_timer;
    PidOptimizer pid_roll_optimizer;
    PidOptimizer pid_pitch_optimizer;
    PidOptimizer pid_yaw_optimizer;

    /* Roll PID */
    float rollPid(float gyro_roll, float roll_desired_angle)
    {
        return constrain(rollPidP(gyro_roll, roll_desired_angle) + roll_pid_i + rollPidD(gyro_roll, roll_desired_angle), -PID_MAX, PID_MAX);
    }

    float rollError(float gyro_roll, float roll_desired_angle)
    {
        return roll_desired_angle - gyro_roll;
    }

    float roll_previous_error = 0;

    float rollPidP(float gyro_roll, float roll_desired_angle)
    {
        return (pid_roll_optimizer.getKp()) * rollError(gyro_roll, roll_desired_angle);
    }

    float roll_pid_i = 0;

    float rollPidD(float gyro_roll, float roll_desired_angle)
    {
        return (pid_roll_optimizer.getKd()) * (rollError(gyro_roll, roll_desired_angle) - roll_previous_error);
    }

    /* Pitch PID */
    float pitchPid(float gyro_pitch, float pitch_desired_angle)
    {
        return constrain(pitchPidP(gyro_pitch, pitch_desired_angle) + pitch_pid_i + pitchPidD(gyro_pitch, pitch_desired_angle), -PID_MAX, PID_MAX);
    }

    float pitchError(float gyro_pitch, float pitch_desired_angle)
    {
        return pitch_desired_angle - gyro_pitch;
    }

    float pitch_previous_error = 0;

    float pitchPidP(float gyro_pitch, float pitch_desired_angle)
    {
        return pid_pitch_optimizer.getKp() * pitchError(gyro_pitch, pitch_desired_angle);
    }

    float pitch_pid_i = 0;

    float pitchPidD(float gyro_pitch, float pitch_desired_angle)
    {
        return pid_pitch_optimizer.getKd() * (pitchError(gyro_pitch, pitch_desired_angle) - pitch_previous_error);
    }

    /* Yaw PID Constants */
    // double yaw_kp = 0.5; // 0.5
    // double yaw_ki = 0.005;
    // double yaw_kd = 2; // 5

    /* Yaw PID */
    float yawPid(float gyro_yaw, float yaw_desired_angle)
    {
        return constrain(yawPidP(gyro_yaw, yaw_desired_angle) + yaw_pid_i + yawPidD(gyro_yaw, yaw_desired_angle), -PID_MAX, PID_MAX);
    }

    float yawError(float gyro_yaw, float yaw_desired_angle)
    {
        return yaw_desired_angle - gyro_yaw;
    }

    float yaw_previous_error = 0;

    float yawPidP(float gyro_yaw, float yaw_desired_angle)
    {
        return pid_yaw_optimizer.getKp() * yawError(gyro_yaw, yaw_desired_angle);
    }

    float yaw_pid_i = 0;

    float yawPidD(float gyro_yaw, float yaw_desired_angle)
    {
        return pid_yaw_optimizer.getKd() * (yawError(gyro_yaw, yaw_desired_angle) - yaw_previous_error);
    }
};

#endif
