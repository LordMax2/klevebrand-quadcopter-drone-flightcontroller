#ifndef KLEVEBRAND_MAXFLY_DRONE_H
#define KLEVEBRAND_MAXFLY_DRONE_H

#include <Arduino.h>
#include "../klevebrand_drone_core/components/base_gyro_drone/base_gyro_drone.h"
#include "./components/quadcopter_pid/quadcopter_pid.h"

class KlevebrandMaxFlyDrone : public BaseGyroDrone<QuadcopterPid>
{
public:
  KlevebrandMaxFlyDrone(
      uint8_t motor_left_front_pin_number,
      uint8_t motor_right_front_pin_number,
      uint8_t motor_left_back_pin_number,
      uint8_t motor_right_back_pin_number) : BaseGyroDrone<QuadcopterPid>(motor_left_front_pin_number, motor_right_front_pin_number, motor_left_back_pin_number, motor_right_back_pin_number) {}
  void setup() override;
  void run() override;
  void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw) override;
};

#endif
