#include <Arduino.h>

#include "trest_drone.h"

Drone drone;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  while (!Serial);

  drone.setup();
}

void loop() {
  drone.gyro.get();

  drone.reciver.recive();

  if(drone.lostConnection()) {
    drone.pid.reset();
    drone.stopMotors();

    Serial.println("LOST CONNECTION");
    
    delay(100);
  } else {
    drone.pid.regulateThrottle();
    drone.setZero();
    drone.calculatePID();
    drone.runMotors();
  }
}
