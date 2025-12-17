#ifndef KLEVEBRAND_MAXJET_DRONE_H
#define KLEVEBRAND_MAXJET_DRONE_H

#include <Arduino.h>
#include "../klevebrand_drone_core/components/base_gyro_drone/base_gyro_drone.h"
#include "./components/airplane_vtail_pid/airplane_vtail_pid.h"

class KlevebrandMaxJetDrone : public BaseGyroDrone<AirplaneVtailPid>
{
public:
    KlevebrandMaxJetDrone(uint8_t (&motor_pin_numbers)[16]) : BaseGyroDrone<AirplaneVtailPid>(motor_pin_numbers) {}
    void setup() override;
    void run() override;
    void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw) override;
};

#endif // KLEVEBRAND_MAXJET_DRONE_H