#include "klevebrand_maxfly_drone.h"

void Drone::setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  if (!Serial)
  {
    Serial.println("Failed to start serial...");
  }
  while (!Serial)
    ;

  Serial.println("Starting drone...");

  gyro.setup();
  setupMotors();

  Serial.println("Drone started!");
}

void Drone::run()
{
  // Get the latest data from the gyroscope
  updateGyro();

  // Check if connection is alive
  if (hasLostConnection())
  {
    // If connection is dead, stop the drone
    resetPid();
    stopMotors();

    Serial.println("LOST CONNECTION");
  }
  else
  {
    // If connection is good, check if we should set the PID zero offset
    setPitchAndRollGyroOffsetAndDefineCurrentAngleAsZero();

    // Then calculate the PID stabilization
    calculatePid();

    // To debug throttle response
    // printThrottle();
    // printGyro();

    // Run the motors with the calculated PID throttle
    runMotors();

    delay(50);
  }
}

void Drone::setupMotors()
{
  Serial.println("Setting up motors...");

  pinMode(motorLeftFrontPinNumber, OUTPUT);
  pinMode(motorRightFrontPinNumber, OUTPUT);
  pinMode(motorLeftBackPinNumber, OUTPUT);
  pinMode(motorRightBackPinNumber, OUTPUT);

  motorLeftFront.attach(motorLeftFrontPinNumber);
  motorRightFront.attach(motorRightFrontPinNumber);
  motorLeftBack.attach(motorLeftBackPinNumber);
  motorRightBack.attach(motorRightBackPinNumber);

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

bool Drone::hasLostConnection()
{
  return millis() - throttleSetTimestamp >= TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS;
}

void Drone::updateGyro()
{
  gyro.update();
}

void Drone::printGyro()
{
  gyro.printYawPitchRoll();
}

void Drone::resetPid()
{
  pid.reset();
}

void Drone::setThrottleYawPitchRollFromReceiver(PwmReceiver receiver)
{
  setThrottle(receiver.getChannelValue(throttleReceiverChannelNumber));
  setDesiredYawAngle(receiver.getChannelValue(yawReceiverChannelNumber));
  setDesiredPitchAngle(receiver.getChannelValue(pitchReceiverChannelNumber));
  setDesiredRollAngle(receiver.getChannelValue(rollReceiverChannelNumber));
}

void Drone::setPIDFromReceiver(PwmReceiver receiver)
{
  setPidPConstant(receiver.getChannelValue(pidPConstantChannelNumber));
  setPidIConstant(receiver.getChannelValue(pidIConstantChannelNumber));
  setPidDConstant(receiver.getChannelValue(pidDConstantChannelNumber));
}

void Drone::setPidPConstant(float pwmValue)
{
  pid.roll_kp = map(pwmValue, 1000, 2000, 0, 100);
}

void Drone::setPidIConstant(float pwmValue)
{
  pid.roll_ki = map(pwmValue, 1000, 2000, 0, 10);
}

void Drone::setPidDConstant(float pwmValue)
{
  pid.roll_kd = map(pwmValue, 1000, 2000, 0, 100);
}

void Drone::setThrottle(float value)
{
  if (value > 1800)
  {
    value = 2000;
  }

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
