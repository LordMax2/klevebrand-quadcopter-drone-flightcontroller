#ifndef KLEVEBRAND_MAXJET_DRONE_H
#define KLEVEBRAND_MAXJET_DRONE_H

#include <Arduino.h>
#include <Servo.h>
#include "./components/airplane_vtail_pid/airplane_vtail_pid.h"
#include "../klevebrand_drone_core/components/gyro/gyro.h"
#include "../klevebrand_drone_core/entities/flight_mode/flight_mode.h"
#include "../klevebrand_drone_core/components/eeprom_pid_repository/eeprom_pid_repository.h"

#define TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS 500

#define SERIAL_BAUD_RATE 115200

#define FEEDBACK_LOOP_HZ 200

#define PID_PERSIST_INTERVAL_MILLISECONDS 10000

class KlevebrandMaxjetDrone
{
private:
    AirplaneVtailPid pid;
    Gyro gyro;
    FlightMode flight_mode;
    EepromPidRepository eeprom_pid_repository;
    Servo servo_left_front;
    Servo servo_right_front;
    Servo servo_left_back;
    Servo servo_right_back;
    float throttle = 0;
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

#endif // KLEVEBRAND_MAXJET_DRONE_H