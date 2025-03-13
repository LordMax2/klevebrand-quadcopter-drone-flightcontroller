#include "../lib/klevebrand_maxfly_drone/klevebrand_maxfly_drone.h"
#include "../lib/klevebrand_maxfly_drone/components/pwm_receiver/pwm_receiver.h"

Drone drone = Drone(6, 7, 2, 3, 1, 2, 3, 4, 5, 6, 7);
PwmReceiver receiver;

void setup()
{
  // Startup the gyroscope and motors
  drone.setup();

  // Startup the reciever
  receiver.setup();
}

void loop()
{
  // Set drone flight values from the receiver
  drone.setThrottleYawPitchRollFromReceiver(receiver);

  // Set the drone PID values from the receiver
  drone.setPIDFromReceiver(receiver);

  // Temp debug print receiver channel one value
  //Serial.println(receiver.getChannelValue(1));

  // Run the drone feedback-loop
  drone.run();
}