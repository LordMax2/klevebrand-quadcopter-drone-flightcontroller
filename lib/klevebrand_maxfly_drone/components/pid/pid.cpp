#include "pid.h"

void Pid::reset()
{
  roll_previous_error = 0, pitch_previous_error = 0, yaw_previous_error = 0;
  yaw_pid_i = 0, roll_pid_i = 0, pitch_pid_i = 0;
}

void Pid::calculate(float throttle, bool launch_mode, float gyro_roll, float gyro_pitch, float gyro_yaw)
{
  if (throttle <= PID_THROTTLE_THRESHOLD)
  {
    reset();
    return;
  }

  unsigned long currentTimer = micros();

  if (currentTimer - previous_timer >= PID_UPDATE_INTERVAL)
  {
    previous_timer = currentTimer;

    if (launch_mode)
    {
      resetIntegral();
    }
    else
    {
      updateIntegral(gyro_roll, gyro_pitch, gyro_yaw);
    }
  }
}



void Pid::updateIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
  roll_pid_i += constrain(roll_ki * roll_error(gyro_roll), -PID_MAX, PID_MAX);
  pitch_pid_i += constrain(pitch_ki * pitch_error(gyro_pitch), -PID_MAX, PID_MAX);
  yaw_pid_i += constrain(yaw_ki * yaw_error(gyro_yaw), -PID_MAX, PID_MAX);
}

void Pid::resetIntegral()
{
  roll_pid_i = 0;
  pitch_pid_i = 0;
  yaw_pid_i = 0;
}