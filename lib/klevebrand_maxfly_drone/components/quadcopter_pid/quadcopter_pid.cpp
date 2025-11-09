#include "quadcopter_pid.h"

float QuadcopterPid::pidThrottleLF(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
{
  if (throttle < PID_THROTTLE_THRESHOLD)
    return THROTTLE_MINIMUM;

  return constrain(throttle + rollPid(gyro_roll, roll_desired_angle) - pitchPid(gyro_pitch, pitch_desired_angle) + yawPid(gyro_yaw, yaw_desired_angle, yaw_compass_mode), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
}

float QuadcopterPid::pidThrottleLB(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
{
  if (throttle < PID_THROTTLE_THRESHOLD)
    return THROTTLE_MINIMUM;

  return constrain(throttle + rollPid(gyro_roll, roll_desired_angle) + pitchPid(gyro_pitch, pitch_desired_angle) - yawPid(gyro_yaw, yaw_desired_angle, yaw_compass_mode), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
}

float QuadcopterPid::pidThrottleRF(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
{
  if (throttle < PID_THROTTLE_THRESHOLD)
    return THROTTLE_MINIMUM;

  return constrain(throttle - rollPid(gyro_roll, roll_desired_angle) - pitchPid(gyro_pitch, pitch_desired_angle) - yawPid(gyro_yaw, yaw_desired_angle, yaw_compass_mode), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
}

float QuadcopterPid::pidThrottleRB(float throttle, float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
{
  if (throttle < PID_THROTTLE_THRESHOLD)
    return THROTTLE_MINIMUM;

  return constrain(throttle - rollPid(gyro_roll, roll_desired_angle) + pitchPid(gyro_pitch, pitch_desired_angle) + yawPid(gyro_yaw, yaw_desired_angle, yaw_compass_mode), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
}