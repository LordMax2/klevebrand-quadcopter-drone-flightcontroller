#include "klevebrand_maxfly_drone.h"

void Drone::setup()
{
  Serial.println("Starting drone...");

  gyro.setup();
  setupMotors();

  Serial.println("Drone started!");
}

void Drone::setupMotors()
{
  Serial.println("Setting up motors...");

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  motorLeftFront.attach(5);
  motorRightFront.attach(6);
  motorLeftBack.attach(7);
  motorRightBack.attach(8);

  motorLeftFront.writeMicroseconds(THROTTLE_MINIMUM);
  motorRightFront.writeMicroseconds(THROTTLE_MINIMUM);
  motorLeftBack.writeMicroseconds(THROTTLE_MINIMUM);
  motorRightBack.writeMicroseconds(THROTTLE_MINIMUM);

  Serial.println("Motors setup!");
}

void Drone::setPitchAndRollGyroOffsetAndDefineCurrentAngleAsZero()
{
  if (setZeroBool)
  {
    pid.pitchOffset = Drone::gyro.pitch();
    pid.rollOffset = Drone::gyro.roll();
    setZeroBool = false;
  }
}

void Drone::stopMotors()
{
  motorLeftFront.writeMicroseconds(THROTTLE_MINIMUM);
  motorRightFront.writeMicroseconds(THROTTLE_MINIMUM);
  motorLeftBack.writeMicroseconds(THROTTLE_MINIMUM);
  motorRightBack.writeMicroseconds(THROTTLE_MINIMUM);
}

void Drone::runMotors()
{
  motorLeftFront.writeMicroseconds(pid.pid_throttle_L_F(throttle));
  motorRightFront.writeMicroseconds(pid.pid_throttle_R_F(throttle));
  motorLeftBack.writeMicroseconds(pid.pid_throttle_L_B(throttle));
  motorRightBack.writeMicroseconds(pid.pid_throttle_R_B(throttle));
}

void Drone::calculatePid()
{
  pid.calculate(throttle, launchMode, gyro.roll(), gyro.pitch(), gyro.yaw());
}

bool Drone::lostConnection()
{
  return millis() - throttleSetTimestamp >= TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS;
}

void Drone::updateGyro()
{
  gyro.update();
}

void Drone::resetPid()
{
  pid.reset();
}

void Drone::setThrottleYawPitchRollFromReceiver(PwmReceiver receiver)
{
  setThrottle(receiver.getChannelValue(1));
  setDesiredYawAngle(receiver.getChannelValue(2));
  setDesiredPitchAngle(receiver.getChannelValue(3));
  setDesiredRollAngle(receiver.getChannelValue(4));
}

void Drone::setThrottle(float value)
{
  throttle = value;
  throttleSetTimestamp = millis();
}

void Drone::setDesiredYawAngle(float value)
{
  desiredYawAngle = value;
  desiredYawAngleSetTimestamp = millis();
}

void Drone::setDesiredPitchAngle(float value)
{
  desiredPitchAngle = value;
  desiredPitchAngleSetTimestamp = millis();
}

void Drone::setDesiredRollAngle(float value)
{
  desiredRollAngle = value;
  desiredRollAngleSetTimestamp = millis();
}

void Drone::printThrottle()
{
  Serial.print(pid.pid_throttle_L_F(throttle));
  Serial.print("    ");
  Serial.println(pid.pid_throttle_R_F(throttle));
  Serial.print(pid.pid_throttle_L_B(throttle));
  Serial.print("    ");
  Serial.println(pid.pid_throttle_R_B(throttle));
  Serial.println("-----------------------------------------");
}
