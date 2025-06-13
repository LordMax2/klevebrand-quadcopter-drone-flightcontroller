#ifndef PID_H
#define PID_H

#include <Arduino.h>

#define PID_THROTTLE_THRESHOLD 1050
#define PID_UPDATE_INTERVAL 10000
#define PID_MAX 400

#define THROTTLE_MINIMUM 1000
#define THROTTLE_MAXIMUM 2000

class Pid
{
public:
    void reset();
    void updateIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw);

    float pidThrottleLF(float throttle, float gyro_roll, float gyro_pitch, float gyro_yaw)
    {
        return constrain(throttle + rollPid(gyro_roll) - pitchPid(gyro_pitch) - yawPid(gyro_yaw), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    float pidThrottleLB(float throttle, float gyro_roll, float gyro_pitch, float gyro_yaw)
    {
        return constrain(throttle + rollPid(gyro_roll) + pitchPid(gyro_pitch) + yawPid(gyro_yaw), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    float pidThrottleRF(float throttle, float gyro_roll, float gyro_pitch, float gyro_yaw)
    {
        return constrain(throttle - rollPid(gyro_roll) - pitchPid(gyro_pitch) + yawPid(gyro_yaw), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    float pidThrottleRB(float throttle, float gyro_roll, float gyro_pitch, float gyro_yaw)
    {
        return constrain(throttle - rollPid(gyro_roll) + pitchPid(gyro_pitch) - yawPid(gyro_yaw), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }

    void setPitchOffset(float value);
    void savePitchError(float gyro_pitch);
    void setRollOffset(float value);
    void saveRollError(float gyro_roll);
    void saveYawError(float gyro_yaw);

    /* Roll PID Constants */
    double roll_kp = 0; // Working: 1.25
    double roll_ki = 0; // Working: 0
    double roll_kd = 0; // Working 25 

    /* Pitch PID Constants */
    double pitch_kp = roll_kp;
    double pitch_ki = roll_ki;
    double pitch_kd = roll_kd;

private:
    long previous_timer;
    float pitch_offset = 0, roll_offset = 0;

    /* Roll PID */
    float rollPid(float gyro_roll)
    {
        return constrain(rollPidP(gyro_roll) + rollPidD(gyro_roll), -PID_MAX, PID_MAX);
    }

    float rollError(float gyro_roll)
    {
        return roll_offset - roll_desired_angle - gyro_roll;
    }

    float roll_previous_error = 0;

    float rollPidP(float gyro_roll)
    {
        return roll_kp * rollError(gyro_roll);
    }

    float roll_pid_i = 0;

    float rollPidD(float gyro_roll)
    {
        return roll_kd * (rollError(gyro_roll) - roll_previous_error);
    }
    float roll_desired_angle = 0;

    /* Pitch PID */
    float pitchPid(float gyro_pitch)
    {
        return constrain(pitchPidP(gyro_pitch) + pitchPidD(gyro_pitch), -PID_MAX, PID_MAX);
    }

    float pitchError(float gyro_pitch)
    {
        return pitch_offset - pitch_desired_angle - gyro_pitch;
    }

    float pitch_previous_error = 0;

    float pitchPidP(float gyro_pitch)
    {
        return pitch_kp * pitchError(gyro_pitch);
    }

    float pitch_pid_i = 0;

    float pitchPidD(float gyro_pitch)
    {
        return pitch_kd * (pitchError(gyro_pitch) - pitch_previous_error);
    }

    float pitch_desired_angle = 0;

    /* Yaw PID Constants */
    double yaw_kp = 0.5; // 0.5
    double yaw_ki = 0;
    double yaw_kd = 5; // 5
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
