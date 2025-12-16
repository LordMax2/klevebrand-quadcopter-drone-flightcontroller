#ifndef KLEVEBRAND_MAXJET_DRONE_H
#define KLEVEBRAND_MAXJET_DRONE_H

#include <Arduino.h>
#include <Servo.h>
#include "../klevebrand_drone_core/components/base_four_motor_drone/base_four_motor_drone.h"
#include "./components/airplane_vtail_pid/airplane_vtail_pid.h"

#define TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS 500

#define SERIAL_BAUD_RATE 115200

#define FEEDBACK_LOOP_HZ 200

#define PID_PERSIST_INTERVAL_MILLISECONDS 10000

class KlevebrandMaxJetDrone: public BaseFourMotorDrone
{
public:
    /*
     * Create a drone
     */
    KlevebrandMaxJetDrone(
        uint8_t motor_left_front_pin_number,
        uint8_t motor_right_front_pin_number,
        uint8_t motor_left_back_pin_number,
        uint8_t motor_right_back_pin_number) : BaseFourMotorDrone(motor_left_front_pin_number, motor_right_front_pin_number, motor_left_back_pin_number, motor_right_back_pin_number),
                                               pid(0, 0, 0, 0, 0, 0, 0, 0, 0) {}
    void setup() override;
    void run() override;
    void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw) override;
    AirplaneVtailPid pid;
    void setPidConstants(float yaw_kp, float yaw_ki, float yaw_kd, float pitch_kp, float pitch_ki, float pitch_kd, float roll_kp, float roll_ki, float roll_kd) override;
};

#endif // KLEVEBRAND_MAXJET_DRONE_H