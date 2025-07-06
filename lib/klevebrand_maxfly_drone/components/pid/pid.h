#ifndef PID_H
#define PID_H

#include <Arduino.h>

#define PID_THROTTLE_THRESHOLD 1100
#define PID_MAX 400

#define THROTTLE_MINIMUM 1000
#define THROTTLE_MAXIMUM 2000

class Pid
{
public:
    void reset();
    void updateIntegral(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw);
    void printPid(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw);

    float pidThrottleLF(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw)
    {
        if (throttle < PID_THROTTLE_THRESHOLD)
            return THROTTLE_MINIMUM;

        return constrain(throttle - rollPid(gyro_roll, roll_desired_angle) + pitchPid(gyro_pitch, pitch_desired_angle) + yawPid(gyro_yaw), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    float pidThrottleLB(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw)
    {
        if (throttle < PID_THROTTLE_THRESHOLD)
            return THROTTLE_MINIMUM;

        return constrain(throttle - rollPid(gyro_roll, roll_desired_angle) - pitchPid(gyro_pitch, pitch_desired_angle) - yawPid(gyro_yaw), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    float pidThrottleRF(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw)
    {
        if (throttle < PID_THROTTLE_THRESHOLD)
            return THROTTLE_MINIMUM;

        return constrain(throttle + rollPid(gyro_roll, roll_desired_angle) + pitchPid(gyro_pitch, pitch_desired_angle) - yawPid(gyro_yaw), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    float pidThrottleRB(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw)
    {
        if (throttle < PID_THROTTLE_THRESHOLD)
            return THROTTLE_MINIMUM;

        return constrain(throttle + rollPid(gyro_roll, roll_desired_angle) - pitchPid(gyro_pitch, pitch_desired_angle) + yawPid(gyro_yaw), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    void setPitchOffset(float value);
    void savePitchError(float gyro_pitch, float pitch_desired_angle);
    void setRollOffset(float value);
    void saveRollError(float gyro_roll, float roll_desired_angle);
    void saveYawError(float gyro_yaw);

    /* Roll PID Constants */
    double roll_kp = 0; // Working: 1.25    (Autolevel)
    double roll_ki = 0; // Working: 0.01    (Autolevel)
    double roll_kd = 0; // Working: 25      (Autolevel)

    /* Pitch PID Constants */
    double pitch_kp = roll_kp;
    double pitch_ki = roll_ki;
    double pitch_kd = roll_kd;

private:
    long previous_timer;
    float pitch_offset = 0, roll_offset = 0;

    /* Roll PID */
    float rollPid(float gyro_roll, float roll_desired_angle)
    {
        return constrain(rollPidP(gyro_roll, roll_desired_angle) + roll_pid_i + rollPidD(gyro_roll, roll_desired_angle), -PID_MAX, PID_MAX);
    }

    float rollError(float gyro_roll, float roll_desired_angle)
    {
        return roll_offset - roll_desired_angle - gyro_roll;
    }

    float roll_previous_error = 0;

    float rollPidP(float gyro_roll, float roll_desired_angle)
    {
        return roll_kp * rollError(gyro_roll, roll_desired_angle);
    }

    float roll_pid_i = 0;

    float rollPidD(float gyro_roll, float roll_desired_angle)
    {
        return roll_kd * (rollError(gyro_roll, roll_desired_angle) - roll_previous_error);
    }

    /* Pitch PID */
    float pitchPid(float gyro_pitch, float pitch_desired_angle)
    {
        return constrain(pitchPidP(gyro_pitch, pitch_desired_angle) + pitch_pid_i + pitchPidD(gyro_pitch, pitch_desired_angle), -PID_MAX, PID_MAX);
    }

    float pitchError(float gyro_pitch, float pitch_desired_angle)
    {
        return pitch_offset - pitch_desired_angle - gyro_pitch;
    }

    float pitch_previous_error = 0;

    float pitchPidP(float gyro_pitch, float pitch_desired_angle)
    {
        return pitch_kp * pitchError(gyro_pitch, pitch_desired_angle);
    }

    float pitch_pid_i = 0;

    float pitchPidD(float gyro_pitch, float pitch_desired_angle)
    {
        return pitch_kd * (pitchError(gyro_pitch, pitch_desired_angle) - pitch_previous_error);
    }

    /* Yaw PID Constants */
    double yaw_kp = 0.1; // 0.5
    double yaw_ki = 0.001;
    double yaw_kd = 1; // 5
    float yaw_desired_angle = 0;

    /* Yaw PID */
    float yawPid(float gyro_yaw)
    {
        return constrain(yawPidP(gyro_yaw) + yaw_pid_i + yawPidD(gyro_yaw), -PID_MAX, PID_MAX);
    }

    float yawError(float gyro_yaw)
    {
        return yaw_desired_angle - gyro_yaw;
    }

    float yaw_previous_error;

    float yawPidP(float gyro_yaw)
    {
        return yaw_kp * yawError(gyro_yaw);
    }

    float yaw_pid_i = 0;

    float yawPidD(float gyro_yaw)
    {
        return yaw_kd * (yawError(gyro_yaw) - yaw_previous_error);
    }
};

#endif
