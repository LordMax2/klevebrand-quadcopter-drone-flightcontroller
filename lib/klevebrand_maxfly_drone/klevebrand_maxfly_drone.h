#ifndef KLEVEBRAND_MAXFLY_DRONE_H
#define KLEVEBRAND_MAXFLY_DRONE_H

#include <Arduino.h>
#include <Servo.h>
#include "./components/pid/pid.h"
#include "./components/gyro/gyro.h"
#include "./components/pwm_receiver/pwm_receiver.h"

#define TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS 500

enum FlightMode_t
{
  acro = 1,
  autoLevel = 2,
  holdPosition = 3
};

class Drone
{
public:
  void setup();
  void setPitchAndRollGyroOffsetAndDefineCurrentAngleAsZero();
  void runMotors();
  void stopMotors();
  void updateGyro();
  void printThrottle();
  void resetPid();
  bool lostConnection();
  void calculatePid();
  void setThrottleYawPitchRollFromReceiver(PwmReceiver receiver);
  void setThrottle(float value);
  void setDesiredYawAngle(float value);
  void setDesiredPitchAngle(float value);
  void setDesiredRollAngle(float value);

private:
  Gyro gyro;
  Pid pid;
  Servo motorLeftFront;
  Servo motorRightFront;
  Servo motorLeftBack;
  Servo motorRightBack;
  bool launchMode = true;
  bool setZeroBool = true;
  float throttle = 0;
  float throttleSetTimestamp = 0;
  float desiredYawAngle = 0;
  float desiredYawAngleSetTimestamp = 0;
  float desiredPitchAngle = 0;
  float desiredPitchAngleSetTimestamp = 0;
  float desiredRollAngle = 0;
  float desiredRollAngleSetTimestamp = 0;
  void setupMotors();
};

#endif
