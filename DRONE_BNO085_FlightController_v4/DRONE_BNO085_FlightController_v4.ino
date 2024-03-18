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

  // Recive data from the reciver
  drone.reciverRecive();

  // Check if connection is alive
  if(drone.lostConnection()) {
    // If connection is dead, stop the drone
    drone.resetPID();
    drone.stopMotors();

    Serial.println("LOST CONNECTION");
    
    delay(100);
  } else {
    // If connection is good, check if we should set the PID zero offset
    drone.setZero();

    // Then calculate the PID stabilization
    drone.calculatePID();

    // regulate the result of the PID calculation to not go over accepted limits of the ESCs
    drone.regulateThrottlePID();

    // Run the motors with the calculated PID throttle
    drone.runMotors();
  }
}
