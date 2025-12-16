#ifndef AIRPLANE_VTAIL_PID_H
#define AIRPLANE_VTAIL_PID_H

#include "../klevebrand_drone_core/components/pid_optimizer/pid_optimizer.h"
#include "../klevebrand_drone_core/components/gyro_pid/gyro_pid.h"

#define PID_THROTTLE_THRESHOLD 1100
#define PID_MAX 400

#define THROTTLE_MINIMUM 1000
#define THROTTLE_MAXIMUM 2000

class AirplaneVtailPid : public GyroPid
{
public:
    AirplaneVtailPid(float yaw_kp, float yaw_ki, float yaw_kd, float pitch_kp, float pitch_ki, float pitch_kd, float roll_kp, float roll_ki, float roll_kd) : GyroPid(yaw_kp, yaw_ki, yaw_kd, pitch_kp, pitch_ki, pitch_kd, roll_kp, roll_ki, roll_kd, PID_MAX) {};

    float pidServoLF(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    float pidServoRF(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    float pidServoLB(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
    float pidServoRB(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode);
};

#endif // AIRPLANE_VTAIL_PID_H