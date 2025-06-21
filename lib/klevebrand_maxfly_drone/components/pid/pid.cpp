#include "pid.h"

void Pid::reset()
{
  roll_previous_error = 0, pitch_previous_error = 0, yaw_previous_error = 0;
  yaw_pid_i = 0, roll_pid_i = 0, pitch_pid_i = 0;
}

void Pid::updateIntegral(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw)
{
  roll_pid_i = constrain(roll_pid_i + (roll_ki * rollError(gyro_roll, roll_desired_angle)), -PID_MAX, PID_MAX);
  pitch_pid_i = constrain(pitch_pid_i + (pitch_ki * pitchError(gyro_pitch, pitch_desired_angle)), -PID_MAX, PID_MAX);
  // yaw_pid_i = constrain(yaw_pid_i + (yaw_ki * yawError(gyro_yaw)), -PID_MAX, PID_MAX);
}

void Pid::setPitchOffset(float value)
{
  pitch_offset = value;
}

void Pid::savePitchError(float gyro_pitch, float pitch_desired_angle)
{
  pitch_previous_error = pitchError(gyro_pitch, pitch_desired_angle);
}

void Pid::setRollOffset(float value)
{
  roll_offset = value;
}

void Pid::saveRollError(float gyro_roll, float roll_desired_angle)
{
  roll_previous_error = rollError(gyro_roll, roll_desired_angle);
}

void Pid::saveYawError(float gyro_yaw)
{
  yaw_previous_error = yawError(gyro_yaw);
}

void Pid::printPid(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw) {
  Serial.print(rollPid(gyro_roll, roll_desired_angle));
  Serial.print(",");
  Serial.print(pitchPid(gyro_pitch, pitch_desired_angle));
  Serial.print(",");
  Serial.println(yawPid(gyro_yaw));
}