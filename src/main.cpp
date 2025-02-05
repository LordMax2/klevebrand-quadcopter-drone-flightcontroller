#include "../lib/klevebrand_maxfly_drone/klevebrand_maxfly_drone.h"
#include "../lib/klevebrand_maxfly_drone/components/pwm_receiver/pwm_receiver.h"

// Declare setup() and loop() functions
void setup();
void loop();

Drone drone;
PwmReceiver receiver;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if(!Serial) {
    Serial.println("Failed to start serial...");
  }
  while (!Serial);

  // Startup the gyro, radio and motors
  drone.setup();

  // Setup the reciever
  receiver.begin();
}

void loop() {
  // Set drone flight values from the receiver
  drone.setThrottleYawPitchRollFromReceiver(receiver);

  Serial.println(receiver.getChannelValue(1));
  
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