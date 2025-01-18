#ifndef PID_H
#define PID_H

#include <Arduino.h>

#define PID_THROTTLE_THRESHOLD 1050
#define PID_UPDATE_INTERVAL 10
#define PID_MAX 400

#define THROTTLE_MINIMUM 1000
#define THROTTLE_MAXIMUM 2000

class Pid
{
public:
    void calculate(float throttle, bool launchMode, float gyroRoll, float gyroPitch, float gyroYaw);
    void reset();
    long previousTimer;
    float pid_throttle_L_F(float throttle) const
    {
        return constrain(throttle + roll_pid() - pitch_pid() - yaw_pid(), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }
    float pid_throttle_L_B(float throttle) const
    {
        return constrain(throttle + roll_pid() + pitch_pid() + yaw_pid(), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }
    float pid_throttle_R_F(float throttle) const
    {
        return constrain(throttle - roll_pid() - pitch_pid() + yaw_pid(), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }
    float pid_throttle_R_B(float throttle) const
    {
        return constrain(throttle - roll_pid() + pitch_pid() - yaw_pid(), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }
    float pitchOffset = 0, rollOffset = 0;

private:
    void updateIntegral();
    void resetIntegral();

    /* Roll PID */
    float roll_pid() const
    {
        return constrain(roll_pid_p() + roll_pid_i + roll_pid_d(), -PID_MAX, PID_MAX);
    }
    float roll_error, roll_previous_error;
    float roll_pid_p() const
    {
        return roll_kp * roll_error;
    }
    float roll_pid_i = 0;
    float roll_pid_d() const
    {
        return roll_kd * (roll_error - roll_previous_error);
    }
    /* Roll PID Constants */
    double roll_kp = 1;
    double roll_ki = 0.01;
    double roll_kd = 10;
    float roll_desired_angle = 0;

    /* Pitch PID */
    float pitch_pid() const
    {
        return constrain(pitch_pid_p() + pitch_pid_i + pitch_pid_d(), -PID_MAX, PID_MAX);
    }
    float pitch_error, pitch_previous_error;
    float pitch_pid_p() const
    {
        return pitch_kp * pitch_error;
    }
    float pitch_pid_i = 0;
    float pitch_pid_d() const
    {
        return pitch_kd * (pitch_error - pitch_previous_error);
    }
    /* Pitch PID Constants */
    double pitch_kp = roll_kp;
    double pitch_ki = roll_ki;
    double pitch_kd = roll_kd;
    float pitch_desired_angle = 0;

    /* Yaw PID */
    float yaw_pid() const
    {
        return constrain(yaw_pid_p() + yaw_pid_i + yaw_pid_d(), -PID_MAX, PID_MAX);
    }
    float yaw_error, yaw_previous_error;
    float yaw_pid_p() const
    {
        return yaw_kp * yaw_error;
    }
    float yaw_pid_i = 0;
    float yaw_pid_d() const
    {
        return yaw_kd * (yaw_error - yaw_previous_error);
    }
    /* Pitch PID Constants */
    double yaw_kp = 0; // 0.5
    double yaw_ki = 0;
    double yaw_kd = 0; // 5
    float yaw_desired_angle = 0;
};

#endif
