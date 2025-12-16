#ifndef BASE_FOUR_MOTOR_DRONE_H
#define BASE_FOUR_MOTOR_DRONE_H

#include <Arduino.h>
#include <Servo.h>
#include "../gyro/gyro.h"
#include "../gyro_pid/gyro_pid.h"
#include "../../entities/flight_mode/flight_mode.h"
#include "../eeprom_pid_repository/eeprom_pid_repository.h"

#define TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS 500

#define SERIAL_BAUD_RATE 115200

#define FEEDBACK_LOOP_HZ 200

#define PID_PERSIST_INTERVAL_MILLISECONDS 10000

#define THROTTLE_MINIMUM 1000

#define PID_MAX 400

class BaseFourMotorDrone
{
public:
  /*
   * Create a drone
   */
  BaseFourMotorDrone(
      uint8_t motor_left_front_pin_number,
      uint8_t motor_right_front_pin_number,
      uint8_t motor_left_back_pin_number,
      uint8_t motor_right_back_pin_number) : pid(0, 0, 0, 0, 0, 0, 0, 0, 0, PID_MAX) 
  {
    /*
     * Map the motor pin numbers
     */
    this->motor_left_front_pin_number = motor_left_front_pin_number;
    this->motor_right_front_pin_number = motor_right_front_pin_number;
    this->motor_left_back_pin_number = motor_left_back_pin_number;
    this->motor_right_back_pin_number = motor_right_back_pin_number;
  }
  virtual void setup();
  virtual void run();
  virtual void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw);
  GyroPid pid;
  virtual void setPidConstants(float yaw_kp, float yaw_ki, float yaw_kd, float pitch_kp, float pitch_ki, float pitch_kd, float roll_kp, float roll_ki, float roll_kd);

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
  void setFlightModeAutoLevel();
  void setFlightModeAcro();
  void enableMotors();
  void disableMotors();
  FlightMode getFlightMode();
  EepromPidRepository eeprom_pid_repository;
  Gyro gyro;
  Servo motor_left_front;
  Servo motor_right_front;
  Servo motor_left_back;
  Servo motor_right_back;
  void setupMotors();
  void calculatePidIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw);
  void stopMotors();
  bool updateGyro();
  void savePidErrors(float gyro_roll, float gyro_pitch, float gyro_yaw);
  void delayToKeepFeedbackLoopHz(long start_micros_timestamp);
  void setFlightMode(FlightMode flight_mode);
  bool isMotorsEnabled();
  void runPidOptimizer();
  void setYawCompassMode(bool yaw_compass_mode);
  void persistPidConstants();
  float throttle = 0;

private:
  FlightMode flight_mode;
  float throttle_set_timestamp = 0;
  float yaw_desired_angle = 0;
  float yaw_desired_angle_set_timestamp = 0;
  bool yaw_compass_mode = false;
  float pitch_desired_angle = 0;
  float desired_pitch_angle_set_timestamp = 0;
  float roll_desired_angle = 0;
  float desired_roll_angle_set_timestamp = 0;
  uint8_t motor_left_front_pin_number;
  uint8_t motor_right_front_pin_number;
  uint8_t motor_left_back_pin_number;
  uint8_t motor_right_back_pin_number;
  bool is_motors_enabled = false;
  unsigned long last_pid_persist_timestamp_milliseconds = 0;
};
;

#endif // BASE_FOUR_MOTOR_DRONE_H