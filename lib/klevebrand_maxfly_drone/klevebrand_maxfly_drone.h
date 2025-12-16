#ifndef KLEVEBRAND_MAXFLY_DRONE_H
#define KLEVEBRAND_MAXFLY_DRONE_H

#include <Arduino.h>
#include "../klevebrand_drone_core/components/base_four_motor_drone/base_four_motor_drone.h"
#include "./components/quadcopter_pid/quadcopter_pid.h"

#define TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS 500

#define SERIAL_BAUD_RATE 115200

#define FEEDBACK_LOOP_HZ 200

#define PID_PERSIST_INTERVAL_MILLISECONDS 10000

class KlevebrandMaxFlyDrone : public BaseFourMotorDrone<QuadcopterPid>
{
public:
  /*
   * Create a drone
   */
  KlevebrandMaxFlyDrone(
      uint8_t motor_left_front_pin_number,
      uint8_t motor_right_front_pin_number,
      uint8_t motor_left_back_pin_number,
      uint8_t motor_right_back_pin_number) : BaseFourMotorDrone<QuadcopterPid>(motor_left_front_pin_number, motor_right_front_pin_number, motor_left_back_pin_number, motor_right_back_pin_number) {}
  void setup() override;
  void run() override;
  void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw) override;
};

#endif
