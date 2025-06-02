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
    void calculate(float throttle, bool launch_mode, float gyro_roll, float gyro_pitch, float gyro_yaw);
    void reset();
    float pid_throttle_L_F(float throttle, float gyro_roll, float gyro_pitch)
    {
        return constrain(throttle + roll_pid(gyro_roll) - pitch_pid(gyro_pitch), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }
    float pid_throttle_L_B(float throttle, float gyro_roll, float gyro_pitch)
    {
        return constrain(throttle + roll_pid(gyro_roll) + pitch_pid(gyro_pitch), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }
    float pid_throttle_R_F(float throttle, float gyro_roll, float gyro_pitch)
    {
        return constrain(throttle - roll_pid(gyro_roll) - pitch_pid(gyro_pitch), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }
    float pid_throttle_R_B(float throttle, float gyro_roll, float gyro_pitch)
    {
        return constrain(throttle - roll_pid(gyro_roll) + pitch_pid(gyro_pitch), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }
    void setPitchOffset(float value) {
        pitch_offset = value;
    };
    
    void save_pitch_error(float gyro_pitch) 
    {
        pitch_previous_error = pitch_error(gyro_pitch);
    }
    void setRollOffset(float value) {
        roll_offset = value;
    };
    void save_roll_error(float gyro_roll) 
    {
        roll_previous_error = roll_error(gyro_roll);
    }

    /* Roll PID Constants */
    double roll_kp = 0;
    double roll_ki = 0;
    double roll_kd = 0;

    /* Pitch PID Constants */
    double pitch_kp = roll_kp;
    double pitch_ki = roll_ki;
    double pitch_kd = roll_kd;
private:
    long previous_timer;
    float pitch_offset = 0, roll_offset = 0;
    
    void updateIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw);
    void resetIntegral();

    /* Roll PID */
    float roll_pid(float gyro_roll)
    {
        return constrain(roll_pid_p(gyro_roll) + roll_pid_d(gyro_roll), -PID_MAX, PID_MAX);
    }
    
    float roll_error(float gyro_roll) 
    {
        return roll_desired_angle - gyro_roll;
    }

    float roll_previous_error = 0;
    
    float roll_pid_p(float gyro_roll)
    {
        return roll_kp * roll_error(gyro_roll);
    }

    float roll_pid_i = 0;
    
    float roll_pid_d(float gyro_roll)
    {
        return roll_kd * (roll_error(gyro_roll) - roll_previous_error);
    }
    float roll_desired_angle = 0;   
    
    /* Pitch PID */
    float pitch_pid(float gyro_pitch)
    {
        return constrain(pitch_pid_p(gyro_pitch) + pitch_pid_d(gyro_pitch), -PID_MAX, PID_MAX);
    }

    float pitch_error(float gyro_pitch) 
    {
        return pitch_desired_angle - gyro_pitch;
    }

    float pitch_previous_error = 0;

    float pitch_pid_p(float gyro_pitch)
    {
        return pitch_kp * pitch_error(gyro_pitch);
    }

    float pitch_pid_i = 0;

    float pitch_pid_d(float gyro_pitch)
    {
        return pitch_kd * (pitch_error(gyro_pitch) - pitch_previous_error);
    }

    float pitch_desired_angle = 0;

    /* Yaw PID Constants */
    double yaw_kp = 0; // 0.5
    double yaw_ki = 0;
    double yaw_kd = 0; // 5
    float yaw_desired_angle = 0;

    /* Yaw PID */
    float yaw_pid()
    {
        return 0;
        //return constrain(yaw_pid_p() + yaw_pid_i + yaw_pid_d(), -PID_MAX, PID_MAX);
    }
    float yaw_error(float gyro_yaw) 
    {
        return yaw_desired_angle - gyro_yaw;
    }
    float yaw_previous_error;
    float yaw_pid_p(float gyro_yaw)
    {
        return yaw_kp * yaw_error(gyro_yaw);
    }
    float yaw_pid_i = 0;
    float yaw_pid_d(float gyro_yaw)
    {
        return yaw_kd * (yaw_error(gyro_yaw) - yaw_previous_error);
    }
};

#endif
