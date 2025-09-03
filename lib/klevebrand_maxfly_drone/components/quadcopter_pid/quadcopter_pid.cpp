#include "quadcopter_pid.h"

void Pid::reset()
{
  roll_previous_error = 0, pitch_previous_error = 0, yaw_previous_error = 0;
  yaw_pid_i = 0, roll_pid_i = 0, pitch_pid_i = 0;
}

void Pid::updateIntegral(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle)
{
  roll_pid_i = constrain(roll_pid_i + (pid_roll_optimizer.getKi() * rollError(gyro_roll, roll_desired_angle)), -PID_MAX, PID_MAX);
  pitch_pid_i = constrain(pitch_pid_i + (pid_pitch_optimizer.getKi() * pitchError(gyro_pitch, pitch_desired_angle)), -PID_MAX, PID_MAX);
  yaw_pid_i = constrain(yaw_pid_i + (pid_yaw_optimizer.getKi() * yawError(gyro_yaw, yaw_desired_angle)), -PID_MAX, PID_MAX);
}

void Pid::runRollOptimizer(float gyro_roll, float roll_desired_angle)
{
  pid_roll_optimizer.run(rollError(gyro_roll, roll_desired_angle));
}

void Pid::runPitchOptimizer(float gyro_pitch, float pitch_desired_angle)
{
  pid_pitch_optimizer.run(pitchError(gyro_pitch, pitch_desired_angle));
}

void Pid::runYawOptimizer(float gyro_yaw, float yaw_desired_angle)
{
  pid_yaw_optimizer.run(yawError(gyro_yaw, yaw_desired_angle));
}

void Pid::savePitchError(float gyro_pitch, float pitch_desired_angle)
{
  pitch_previous_error = pitchError(gyro_pitch, pitch_desired_angle);
}

void Pid::saveRollError(float gyro_roll, float roll_desired_angle)
{
  roll_previous_error = rollError(gyro_roll, roll_desired_angle);
}

void Pid::saveYawError(float gyro_yaw, float yaw_desired_angle)
{
  yaw_previous_error = yawError(gyro_yaw, yaw_desired_angle);
}

void Pid::printPid(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle)
{
  Serial.print(rollPid(gyro_roll, roll_desired_angle));
  Serial.print(",");
  Serial.print(pitchPid(gyro_pitch, pitch_desired_angle));
  Serial.print(",");
  Serial.println(yawPid(gyro_yaw, yaw_desired_angle));
}

void Pid::printPidConstants()
{
  Serial.print(pid_roll_optimizer.getKp());
  Serial.print(",");
  Serial.print(pid_roll_optimizer.getKi());
  Serial.print(",");
  Serial.println(pid_roll_optimizer.getKd());
}
