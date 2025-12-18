#ifndef BASE_FOUR_MOTOR_DRONE_H
#define BASE_FOUR_MOTOR_DRONE_H

#include <Arduino.h>
#include <Servo.h>
#include "./gyro.h"
#include "./gyro_pid.h"
#include "./flight_mode.h"
#include "./eeprom_pid_repository.h"

#define TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS 500

#define SERIAL_BAUD_RATE 115200

#define FEEDBACK_LOOP_HZ 200

#define PID_PERSIST_INTERVAL_MILLISECONDS 10000

#define THROTTLE_MINIMUM 1000

#define PID_MAX 400

template <class SomeGyroPidType>
class BaseGyroDrone
{
public:
  /*
   * Create a drone
   */
  BaseGyroDrone(uint8_t (&motor_pin_numbers)[16]) : pid(0, 0, 0, 0, 0, 0, 0, 0, 0)
  {
    this->motor_pin_numbers = motor_pin_numbers;
  }
  virtual void setup() {};
  virtual void run() {};
  virtual void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw) {};
  SomeGyroPidType pid;

  void setPidConstants(float yaw_kp, float yaw_ki, float yaw_kd, float pitch_kp, float pitch_ki, float pitch_kd, float roll_kp, float roll_ki, float roll_kd)
  {
    pid = SomeGyroPidType(yaw_kp, yaw_ki, yaw_kd, pitch_kp, pitch_ki, pitch_kd, roll_kp, roll_ki, roll_kd);
  };
  void printGyro()
  {
    gyro.printYawPitchRoll();
  }
  void printPid()
  {
    pid.printPid(gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle, yaw_compass_mode);
  }
  void printPidConstants()
  {
    pid.printConstants();
  }
  void resetPid()
  {
    pid.reset();
  }
  bool hasLostConnection()
  {
    bool transmitter_lost_connection = millis() - throttle_set_timestamp >= TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS;

    bool gyro_lost_connection = millis() - gyro.timestamp_milliseconds() >= TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS;

    return transmitter_lost_connection || gyro_lost_connection;
  }
  void setThrottle(float value)
  {
    throttle = value;
    throttle_set_timestamp = millis();
  }
  void setDesiredYawAngle(float value)
  {
    yaw_desired_angle = value;
    yaw_desired_angle_set_timestamp = millis();
  }
  void setDesiredPitchAngle(float value)
  {
    pitch_desired_angle = value;
    desired_pitch_angle_set_timestamp = millis();
  }
  void setDesiredRollAngle(float value)
  {
    roll_desired_angle = value;
    desired_roll_angle_set_timestamp = millis();
  }
  void setFlightModeAutoLevel()
  {
    // Temprorary return early util I have connected the IMU's reset pin
    if (getFlightMode() == auto_level)
      return;

    setFlightMode(auto_level);

    gyro.reset();

    delay(1000);

    gyro.setReportModeEuler();

    PidConstants pid_constants = eeprom_pid_repository.get(256);

    if (pid_constants.isValid())
    {
      setPidConstants(pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd,
                      pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
                      pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);

      pid_constants.print();
    }
    else
    {
      setPidConstants(1.25, 0.01, 25, 1.25, 0.01, 25, 0.5, 0.005, 2);
    }

    setYawCompassMode(true);

    Serial.println("FLIGHT MODE AUTOLEVEL");
  }
  void setFlightModeAcro()
  {
    // Temprorary return early util I have connected the IMU's reset pin
    if (getFlightMode() == acro)
      return;

    setFlightMode(acro);

    gyro.reset();

    delay(1000);

    gyro.setReportModeAcro();

    PidConstants pid_constants = eeprom_pid_repository.get(128);

    if (pid_constants.isValid())
    {
      setPidConstants(pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd,
                      pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
                      pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);

      pid_constants.print();
    }
    else
    {
      setPidConstants(0.4, 0.02, 6, 0.4, 0.02, 6, 0.4, 0.02, 6);
    }

    setYawCompassMode(false);

    Serial.println("FLIGHT MODE ACRO");
  }
  void enableMotors()
  {
    is_motors_enabled = true;
  }
  void disableMotors()
  {
    is_motors_enabled = false;
  }
  FlightMode getFlightMode()
  {
    return flight_mode;
  }
  EepromPidRepository eeprom_pid_repository;
  Gyro gyro;
  Servo* motors;
  void setupMotors()
  {
    Serial.println("SETTING UP MOTORS...");

    for(int i = 0; i < 16; i++) {
      if(motor_pin_numbers[i] <= 0) continue;

      pinMode(motor_pin_numbers[i], OUTPUT);

      Servo motor;
      motors[motor_pin_numbers[i]] = motor;
      motors[motor_pin_numbers[i]].attach(motor_pin_numbers[i]);
      motors[motor_pin_numbers[i]].writeMicroseconds(THROTTLE_MINIMUM);
    }

    delay(1000);

    Serial.println("MOTORS SETUP!");
  }
  void calculatePidIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw)
  {
    pid.updateIntegral(gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode);
  }
  void stopMotors()
  {
    for(int i = 0; i < 16; i++) {
      if(motor_pin_numbers[i] <= 0) continue;

      motors[motor_pin_numbers[i]].writeMicroseconds(THROTTLE_MINIMUM);
    }
  }
  bool updateGyro()
  {
    return gyro.reload();
  }
  void savePidErrors(float gyro_roll, float gyro_pitch, float gyro_yaw)
  {
    pid.savePitchError(gyro_pitch, pitch_desired_angle);
    pid.saveRollError(gyro_roll, roll_desired_angle);
    pid.saveYawError(gyro_yaw, yaw_desired_angle, yaw_compass_mode);
  }
  void delayToKeepFeedbackLoopHz(long start_micros_timestamp)
  {
    long current_micros_timestamp = micros();

    long microseconds_feedback_loop_should_take = 1000000 / FEEDBACK_LOOP_HZ;

    long microseconds_left_for_loop = microseconds_feedback_loop_should_take - (current_micros_timestamp - start_micros_timestamp);

    if (microseconds_left_for_loop > 0 && microseconds_left_for_loop < microseconds_feedback_loop_should_take)
    {
      delayMicroseconds(microseconds_left_for_loop);
    }
  }
  void setFlightMode(FlightMode flight_mode)
  {
    BaseGyroDrone::flight_mode = flight_mode;
  }
  bool isMotorsEnabled()
  {
    return is_motors_enabled;
  }
  void runPidOptimizer()
  {
    pid.runRollOptimizer(gyro.roll(), roll_desired_angle);
    pid.runPitchOptimizer(gyro.pitch(), pitch_desired_angle);
    pid.runYawOptimizer(gyro.yaw(), yaw_desired_angle, yaw_compass_mode);
  }

  void setYawCompassMode(bool yaw_compass_mode)
  {
    this->yaw_compass_mode = yaw_compass_mode;
  }
  void persistPidConstants()
  {
    if (millis() - last_pid_persist_timestamp_milliseconds >= PID_PERSIST_INTERVAL_MILLISECONDS)
    {
      int address = 128;

      switch (getFlightMode())
      {
      case acro:
        address = 128;
        break;
      case auto_level:
        address = 256;
        break;
      }

      PidConstants pid_constants = PidConstants(
          pid.getYawKp(), pid.getYawKi(), pid.getYawKd(),
          pid.getPitchKp(), pid.getPitchKi(), pid.getPitchKd(),
          pid.getRollKp(), pid.getRollKi(), pid.getRollKd());

      eeprom_pid_repository.save(pid_constants, address);

      last_pid_persist_timestamp_milliseconds = millis();
    }
  };
  float throttle = 0;
  float yaw_desired_angle = 0;
  float pitch_desired_angle = 0;
  float roll_desired_angle = 0;
  bool yaw_compass_mode = false;

private:
  FlightMode flight_mode;
  float throttle_set_timestamp = 0;
  float yaw_desired_angle_set_timestamp = 0;
  float desired_pitch_angle_set_timestamp = 0;
  float desired_roll_angle_set_timestamp = 0;
  uint8_t* motor_pin_numbers;
  bool is_motors_enabled = false;
  unsigned long last_pid_persist_timestamp_milliseconds = 0;
};

#endif // BASE_FOUR_MOTOR_DRONE_H