#include "gyro_pid.h"

void GyroPid::reset()
{
  roll_previous_error = 0, pitch_previous_error = 0, yaw_previous_error = 0;
  yaw_pid_i = 0, roll_pid_i = 0, pitch_pid_i = 0;
}

void GyroPid::updateIntegral(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
{
  // Since we always run this at the same frequency, we dont need the time between the measurements
  roll_pid_i = constrain(roll_pid_i + (pid_roll_optimizer.getKi() * rollError(gyro_roll, roll_desired_angle)), -pid_max, pid_max);
  pitch_pid_i = constrain(pitch_pid_i + (pid_pitch_optimizer.getKi() * pitchError(gyro_pitch, pitch_desired_angle)), -pid_max, pid_max);
  yaw_pid_i = constrain(yaw_pid_i + (pid_yaw_optimizer.getKi() * yawError(gyro_yaw, yaw_desired_angle, yaw_compass_mode)), -pid_max, pid_max);
}

void GyroPid::runRollOptimizer(float gyro_roll, float roll_desired_angle)
{
  pid_roll_optimizer.run(rollError(gyro_roll, roll_desired_angle));
}

void GyroPid::runPitchOptimizer(float gyro_pitch, float pitch_desired_angle)
{
  pid_pitch_optimizer.run(pitchError(gyro_pitch, pitch_desired_angle));
}

void GyroPid::runYawOptimizer(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
{
  pid_yaw_optimizer.run(yawError(gyro_yaw, yaw_desired_angle, yaw_compass_mode));
}

void GyroPid::savePitchError(float gyro_pitch, float pitch_desired_angle)
{
  pitch_previous_error = pitchError(gyro_pitch, pitch_desired_angle);
}

void GyroPid::saveRollError(float gyro_roll, float roll_desired_angle)
{
  roll_previous_error = rollError(gyro_roll, roll_desired_angle);
}

void GyroPid::saveYawError(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
{
  yaw_previous_error = yawError(gyro_yaw, yaw_desired_angle, yaw_compass_mode);
}

void GyroPid::printPid(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
{
  Serial.print(rollPid(gyro_roll, roll_desired_angle));
  Serial.print(",");
  Serial.print(pitchPid(gyro_pitch, pitch_desired_angle));
  Serial.print(",");
  Serial.println(yawPid(gyro_yaw, yaw_desired_angle, yaw_compass_mode));
}

void GyroPid::printPidConstants()
{
  Serial.print(pid_roll_optimizer.getKp());
  Serial.print(",");
  Serial.print(pid_roll_optimizer.getKi());
  Serial.print(",");
  Serial.println(pid_roll_optimizer.getKd());
}

float GyroPid::rollPid(float gyro_roll, float roll_desired_angle)
{
  return constrain(rollPidP(gyro_roll, roll_desired_angle) + roll_pid_i + rollPidD(gyro_roll, roll_desired_angle), -pid_max, pid_max);
}

float GyroPid::rollError(float gyro_roll, float roll_desired_angle)
{
  return roll_desired_angle - gyro_roll;
}

float GyroPid::rollPidP(float gyro_roll, float roll_desired_angle)
{
  return (pid_roll_optimizer.getKp()) * rollError(gyro_roll, roll_desired_angle);
}

float GyroPid::rollPidD(float gyro_roll, float roll_desired_angle)
{
  return (pid_roll_optimizer.getKd()) * (rollError(gyro_roll, roll_desired_angle) - roll_previous_error);
}

float GyroPid::pitchPid(float gyro_pitch, float pitch_desired_angle)
{
  return constrain(pitchPidP(gyro_pitch, pitch_desired_angle) + pitch_pid_i + pitchPidD(gyro_pitch, pitch_desired_angle), -pid_max, pid_max);
}

float GyroPid::pitchError(float gyro_pitch, float pitch_desired_angle)
{
  return pitch_desired_angle - gyro_pitch;
}

float GyroPid::pitchPidP(float gyro_pitch, float pitch_desired_angle)
{
  return pid_pitch_optimizer.getKp() * pitchError(gyro_pitch, pitch_desired_angle);
}

float GyroPid::pitchPidD(float gyro_pitch, float pitch_desired_angle)
{
  return pid_pitch_optimizer.getKd() * (pitchError(gyro_pitch, pitch_desired_angle) - pitch_previous_error);
}

float GyroPid::yawPid(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
{
  return constrain(yawPidP(gyro_yaw, yaw_desired_angle, yaw_compass_mode) + yawPidD(gyro_yaw, yaw_desired_angle, yaw_compass_mode), -pid_max, pid_max);
}

float GyroPid::yawError(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
{
  if (yaw_compass_mode)
  {
    auto error = fmod((gyro_yaw + 180), 360) - fmod((yaw_desired_angle + 180), 360);
    float absolute_error = min(abs(error), 360 - abs(error));

    if (error < 0)
    {
      return -1 * absolute_error;
    }

    return absolute_error;
  }

  return yaw_desired_angle - gyro_yaw;
}

float GyroPid::yawPidP(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
{
  return pid_yaw_optimizer.getKp() * yawError(gyro_yaw, yaw_desired_angle, yaw_compass_mode);
}

float yaw_pid_i = 0;

float GyroPid::yawPidD(float gyro_yaw, float yaw_desired_angle, bool yaw_compass_mode)
{
  return pid_yaw_optimizer.getKd() * (yawError(gyro_yaw, yaw_desired_angle, yaw_compass_mode) - yaw_previous_error);
}