#ifndef KLEVEBRAND_MAXFLY_DRONE_H
#define KLEVEBRAND_MAXFLY_DRONE_H

#include <Arduino.h>
#include <Servo.h>
#include "./components/pid/pid.h"
#include "./components/gyro/gyro.h"
#include "./components/pwm_receiver/pwm_receiver.h"

#define TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS 500

#define SERIAL_BAUD_RATE 115200

enum FlightMode_t
{
  acro = 1,
  autoLevel = 2,
  holdPosition = 3
};

class Drone
{
public:
  /*
  * This constructor is here to force the user to configure the drone properly
  */
  Drone(
      uint8_t motorLeftFrontPinNumber,
      uint8_t motorRightFrontPinNumber,
      uint8_t motorLeftBackPinNumber,
      uint8_t motorRightBackPinNumber,
      int throttleReceiverChannelNumber,
      int yawReceiverChannelNumber,
      int pitchReceiverChannelNumber,
      int rollReceiverChannelNumber)
  {
    /*
    * Map the motor pin numbers
    */
    this->motorLeftFrontPinNumber = motorLeftFrontPinNumber;
    this->motorRightFrontPinNumber = motorRightFrontPinNumber;
    this->motorLeftBackPinNumber = motorLeftBackPinNumber;
    this->motorRightBackPinNumber = motorRightBackPinNumber;

    /*
    * Map the receiver channel numbers
    */
    this->throttleReceiverChannelNumber = throttleReceiverChannelNumber;
    this->yawReceiverChannelNumber = yawReceiverChannelNumber;
    this->pitchReceiverChannelNumber = pitchReceiverChannelNumber;
    this->rollReceiverChannelNumber = rollReceiverChannelNumber;
  };
  void setup();
  void run();
  void setPitchAndRollGyroOffsetAndDefineCurrentAngleAsZero();
  void runMotors();
  void stopMotors();
  void updateGyro();
  void printGyro();
  void printThrottle();
  void resetPid();
  bool hasLostConnection();
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
  uint8_t motorLeftFrontPinNumber;
  uint8_t motorRightFrontPinNumber;
  uint8_t motorLeftBackPinNumber;
  uint8_t motorRightBackPinNumber;
  int throttleReceiverChannelNumber;
  int yawReceiverChannelNumber;
  int pitchReceiverChannelNumber;
  int rollReceiverChannelNumber;
  void setupMotors();
};

#endif
