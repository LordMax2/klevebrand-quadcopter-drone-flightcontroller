#ifndef KLEVEBRAND_MAXFLY_DRONE_H
#define KLEVEBRAND_MAXFLY_DRONE_H

#include <Arduino.h>
#include <Servo.h>
#include "./components/pid/pid.h"
#include "./components/gyro/gyro.h"
#include "./components/flight_mode/flight_mode.h"

#define TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS 500

#define SERIAL_BAUD_RATE 115200

#define FEEDBACK_LOOP_HZ 200

class Drone
{
public:
  /*
   * Create a drone
   */
  Drone(
      uint8_t motor_left_front_pin_number,
      uint8_t motor_right_front_pin_number,
      uint8_t motor_left_back_pin_number,
      uint8_t motor_right_back_pin_number)
  {
    /*
     * Map the motor pin numbers
     */
    this->motor_left_front_pin_number = motor_left_front_pin_number;
    this->motor_right_front_pin_number = motor_right_front_pin_number;
    this->motor_left_back_pin_number = motor_left_back_pin_number;
    this->motor_right_back_pin_number = motor_right_back_pin_number;
  }
  void setup();
  void run();
  void printGyro();
  void printThrottle();
  void printPid();
  void printPidConstants();
  void resetPid();
  bool hasLostConnection();
  void setThrottle(float value);
  void setDesiredYawAngle(float value);
  void setDesiredPitchAngle(float value);
  void setDesiredRollAngle(float value);
  void setPidPConstant(float value);
  void setPidIConstant(float value);
  void setPidDConstant(float value);
  void setFlightModeAutoLevel();
  void setFlightModeAcro();
  FlightMode getFlightMode();

private:
  FlightMode flight_mode;
  Gyro gyro;
  Pid pid;
  Servo motor_left_front;
  Servo motor_right_front;
  Servo motor_left_back;
  Servo motor_right_back;
  float throttle = 0;
  float throttle_set_timestamp = 0;
  float yaw_desired_angle = 0;
  float yaw_desired_angle_set_timestamp = 0;
  float pitch_desired_angle = 0;
  float desired_pitch_angle_set_timestamp = 0;
  float roll_desired_angle = 0;
  float desired_roll_angle_set_timestamp = 0;
  uint8_t motor_left_front_pin_number;
  uint8_t motor_right_front_pin_number;
  uint8_t motor_left_back_pin_number;
  uint8_t motor_right_back_pin_number;
  void setupMotors();
  void calculatePidIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw);
  void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw);
  void stopMotors();
  bool updateGyro();
  void savePidErrors(float gyro_roll, float gyro_pitch, float gyro_yaw);
  void delayToKeepFeedbackLoopHz(long start_micros_timestamp);
  void setFlightMode(FlightMode flight_mode);
};

#endif
