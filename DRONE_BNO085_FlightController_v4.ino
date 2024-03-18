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
  drone.updateGyro();

  drone.reciverRecive();

  if(drone.lostConnection()) {
    drone.resetPID();
    drone.stopMotors();

    Serial.println("LOST CONNECTION");
    
    delay(100);
  } else {
    drone.regulateThrottlePID();
    drone.setZero();
    drone.calculatePID();
    drone.runMotors();
  }
}
