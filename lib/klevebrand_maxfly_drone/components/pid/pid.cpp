#include "pid.h"

void Pid::reset()
{
  roll_previous_error = 0, pitch_previous_error = 0, yaw_previous_error = 0;
  yaw_pid_i = 0, roll_pid_i = 0, pitch_pid_i = 0;
}

void Pid::updateIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
  roll_pid_i += constrain(roll_ki * rollError(gyro_roll), -PID_MAX, PID_MAX);
  pitch_pid_i += constrain(pitch_ki * pitchError(gyro_pitch), -PID_MAX, PID_MAX);
  yaw_pid_i += constrain(yaw_ki * yawError(gyro_yaw), -PID_MAX, PID_MAX);
}
