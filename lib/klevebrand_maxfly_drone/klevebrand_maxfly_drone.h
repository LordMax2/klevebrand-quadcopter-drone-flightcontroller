#ifndef KLEVEBRAND_MAXFLY_DRONE_H
#define KLEVEBRAND_MAXFLY_DRONE_H

#include <Arduino.h>
#include <Servo.h>
#include "./components/pid/pid.h"
#include "./components/gyro/gyro.h"
#include "./components/pwm_receiver/pwm_receiver.h"

#define TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS 500

#define SERIAL_BAUD_RATE 115200

#define FEEDBACK_LOOP_HZ 200

enum FlightMode_t
{
  acro = 1,
  auto_level = 2,
  hold_position = 3
};

class Drone
{
public:
  Drone(
      uint8_t motor_left_front_pin_number,
      uint8_t motor_right_front_pin_number,
      uint8_t motor_left_back_pin_number,
      uint8_t motor_right_back_pin_number,
      int throttle_receiver_channel_number,
      int yaw_receiver_channel_number,
      int pitch_receiver_channel_number,
      int roll_receiver_channel_number)
  {
    /*
     * Map the motor pin numbers
     */
    this->motor_left_front_pin_number = motor_left_front_pin_number;
    this->motor_right_front_pin_number = motor_right_front_pin_number;
    this->motor_left_back_pin_number = motor_left_back_pin_number;
    this->motor_right_back_pin_number = motor_right_back_pin_number;

    /*
     * Map the reciever PID channel numbers
     */
    this->pid_p_constant_channel_number = pid_p_constant_channel_number;
    this->pid_i_constant_channel_number = pid_i_constant_channel_number;
    this->pid_d_constant_channel_number = pid_d_constant_channel_number;

  }
  /*
   * This constructor is here to force the user to configure the drone properly
   */
  Drone(
      uint8_t motor_left_front_pin_number,
      uint8_t motor_right_front_pin_number,
      uint8_t motor_left_back_pin_number,
      uint8_t motor_right_back_pin_number,
      int throttle_receiver_channel_number,
      int yaw_receiver_channel_number,
      int pitch_receiver_channel_number,
      int roll_receiver_channel_number,
      int pid_p_constant_channel_number,
      int pid_i_constant_channel_number,
      int pid_d_constant_channel_number) : Drone(
        motor_left_front_pin_number, 
        motor_right_front_pin_number, 
        motor_left_back_pin_number, 
        motor_right_back_pin_number,  
        throttle_receiver_channel_number, 
        yaw_receiver_channel_number, 
        pitch_receiver_channel_number, 
        roll_receiver_channel_number)
  {
    /*
     * Map the receiver channel numbers
     */
    this->throttle_receiver_channel_number = throttle_receiver_channel_number;
    this->yaw_receiver_channel_number = yaw_receiver_channel_number;
    this->pitch_receiver_channel_number = pitch_receiver_channel_number;
    this->roll_receiver_channel_number = roll_receiver_channel_number;
  };
  void setup();
  void run();
  void setPitchAndRollGyroOffsetAndDefineCurrentAngleAsZero();
  void printGyro();
  void printThrottle();
  void resetPid();
  bool hasLostConnection();
  void setThrottleYawPitchRollFromReceiver(PwmReceiver receiver);
  void setPidFromReceiver(PwmReceiver receiver);
  void setThrottle(float value);
  void setDesiredYawAngle(float value);
  void setDesiredPitchAngle(float value);
  void setDesiredRollAngle(float value);
  void setPidPConstant(float pwm_value);
  void setPidIConstant(float pwm_value);
  void setPidDConstant(float pwm_value);

private:
  Gyro gyro;
  Pid pid;
  Servo motor_left_front;
  Servo motor_right_front;
  Servo motor_left_back;
  Servo motor_right_back;
  bool launch_mode = true;
  bool set_zero_bool = true;
  float throttle = 0;
  float throttle_set_timestamp = 0;
  float desired_yaw_angle = 0;
  float desired_yaw_angle_set_timestamp = 0;
  float desired_pitch_angle = 0;
  float desired_pitch_angle_set_timestamp = 0;
  float desired_roll_angle = 0;
  float desired_roll_angle_set_timestamp = 0;
  uint8_t motor_left_front_pin_number;
  uint8_t motor_right_front_pin_number;
  uint8_t motor_left_back_pin_number;
  uint8_t motor_right_back_pin_number;
  int throttle_receiver_channel_number;
  int yaw_receiver_channel_number;
  int pitch_receiver_channel_number;
  int roll_receiver_channel_number;
  int pid_p_constant_channel_number;
  int pid_i_constant_channel_number;
  int pid_d_constant_channel_number;
  void setupMotors();
  void calculatePid(float gyro_roll, float gyro_pitch, float gyro_yaw);
  void runMotors(float gyro_roll, float gyro_pitch);
  void stopMotors();
  void updateGyro();
  void savePidErrors(float gyro_roll, float gyro_pitch)
  {
    pid.save_pitch_error(gyro_pitch);
    pid.save_roll_error(gyro_roll);
  }
};

#endif
