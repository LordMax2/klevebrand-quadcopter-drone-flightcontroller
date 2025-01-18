#include <Arduino.h>
#include "klevebrand_maxfly_drone.h"

// Define the drone
Drone drone;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  while (!Serial);

  // Startup the gyro, radio and motors
  drone.setup();
}

void loop() {
  // Get the latest data from the gyroscope
  drone.updateGyro();

  // Check if connection is alive
  if(drone.lostConnection()) {
    // If connection is dead, stop the drone
    drone.resetPid();
    drone.stopMotors();

    Serial.println("LOST CONNECTION");
  } else {
    // If connection is good, check if we should set the PID zero offset
    drone.setPitchAndRollGyroOffsetAndDefineCurrentAngleAsZero();

    // Then calculate the PID stabilization
    drone.calculatePid();

    // To debug throttle response
    //drone.printThrottle();

    // Run the motors with the calculated PID throttle
    drone.runMotors();
  }
}
