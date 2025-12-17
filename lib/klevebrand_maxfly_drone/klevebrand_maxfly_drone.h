#ifndef KLEVEBRAND_MAXFLY_DRONE_H
#define KLEVEBRAND_MAXFLY_DRONE_H

#include <Arduino.h>
#include "../klevebrand_drone_core/components/base_gyro_drone/base_gyro_drone.h"
#include "./components/quadcopter_pid/quadcopter_pid.h"

class KlevebrandMaxFlyDrone : public BaseGyroDrone<QuadcopterPid>
{
public:
  KlevebrandMaxFlyDrone(uint8_t (&motor_pin_numbers)[16]) : BaseGyroDrone<QuadcopterPid>(motor_pin_numbers) {}
  void setup() override;
  void run() override;
  void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw) override;

private:
  Servo motorLeftFront() { return motors[0]; };
  Servo motorRightFront() { return motors[1]; };
  Servo motorLeftBack() { return motors[2]; };
  Servo motorRightBack() { return motors[3]; };
};

#endif
