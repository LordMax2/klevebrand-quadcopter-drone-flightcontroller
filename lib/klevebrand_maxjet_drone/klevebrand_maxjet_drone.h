#ifndef KLEVEBRAND_MAXJET_DRONE_H
#define KLEVEBRAND_MAXJET_DRONE_H

#include <Arduino.h>
#include <Servo.h>
#include "./components/airplane_vtail_pid/airplane_vtail_pid.h"

class KlevebrandMaxjetDrone
{
private:
    AirplaneVtailPid pid;
};

#endif // KLEVEBRAND_MAXJET_DRONE_H