#include "../lib/klevebrand_maxfly_drone/klevebrand_maxfly_drone.h"
#include "../lib/klevebrand_maxfly_drone/components/pwm_receiver/pwm_receiver.h"
#include "../lib/klevebrand_maxfly_drone/components/drone_pwm_receiver/drone_pwm_receiver.h"

Drone drone = Drone(3, 2, 7, 6);
DronePwmReceiver receiver = DronePwmReceiver(1, 4, 3, 2, 5, 6, 8);

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
  receiver.setThrottleYawPitchRoll(&drone);

  // Set the drone PID values from the receiver
  // receiver.setPid(&drone);

  // Run the drone feedback-loop
  drone.run();
}
