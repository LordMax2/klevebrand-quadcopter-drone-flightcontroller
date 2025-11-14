#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "../pid_optimizer/pid_optimizer.h"
#include "../gyro_pid/gyro_pid.h"

#define PID_THROTTLE_THRESHOLD 1100
#define PID_MAX 400

#define THROTTLE_MINIMUM 1000
#define THROTTLE_MAXIMUM 2000

class QuadcopterPid : public GyroPid
{
public:
    QuadcopterPid(float yaw_kp, float yaw_ki, float yaw_kd, float pitch_kp, float pitch_ki, float pitch_kd, float roll_kp, float roll_ki, float roll_kd) : GyroPid(yaw_kp, yaw_ki, yaw_kd, pitch_kp, pitch_ki, pitch_kd, roll_kp, roll_ki, roll_kd, PID_MAX) {};

    float pidThrottleLF(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    float pidThrottleLB(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    float pidThrottleRF(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    float pidThrottleRB(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
};
#endif
