#include "pid.h"

void Pid::reset()
{
  roll_error = 0, pitch_error = 0, yaw_error = 0;
  roll_previous_error = 0, pitch_previous_error = 0, yaw_previous_error = 0;
  yaw_pid_i = 0, roll_pid_i = 0, pitch_pid_i = 0;
}

void Pid::calculate(float throttle, bool launchMode, float gyroRoll, float gyroPitch, float gyroYaw)
{
  if (throttle <= PID_THROTTLE_THRESHOLD)
  {
    reset();
    return;
  }

  roll_error = roll_desired_angle - gyroRoll;
  pitch_error = pitch_desired_angle - gyroPitch;
  yaw_error = yaw_desired_angle - gyroYaw;

  unsigned long currentTimer = millis();
  if (currentTimer - previousTimer >= PID_UPDATE_INTERVAL)
  {
    previousTimer = currentTimer;

    if (launchMode)
    {
      resetIntegral();
    }
    else
    {
      updateIntegral();
    }

    roll_previous_error = roll_error;
    pitch_previous_error = pitch_error;
    yaw_previous_error = yaw_error;
  }
}

void Pid::updateIntegral()
{
  roll_pid_i += constrain(roll_ki * roll_error, -PID_MAX, PID_MAX);
  pitch_pid_i += constrain(pitch_ki * pitch_error, -PID_MAX, PID_MAX);
  yaw_pid_i += constrain(yaw_ki * yaw_error, -PID_MAX, PID_MAX);
}

void Pid::resetIntegral()
{
  roll_pid_i = 0;
  pitch_pid_i = 0;
  yaw_pid_i = 0;
}