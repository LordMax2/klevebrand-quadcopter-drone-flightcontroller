#include "../lib/klevebrand_maxfly_drone/klevebrand_maxfly_drone.h"
#include "../lib/klevebrand_maxjet_drone/klevebrand_maxjet_drone.h"
#include "../lib/klevebrand_drone_core/components/pwm_receiver/pwm_receiver.h"
#include "../lib/klevebrand_maxfly_drone/components/drone_pwm_receiver/drone_pwm_receiver.h"

uint8_t motor_pin_numbers[16] = { 3, 2, 7, 6};

KlevebrandMaxFlyDrone drone = KlevebrandMaxFlyDrone(motor_pin_numbers);
//KlevebrandMaxJetDrone drone2 = KlevebrandMaxJetDrone(motor_pin_numbers);
DronePwmReceiver receiver = DronePwmReceiver(1, 4, 3, 2, 7);

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

  // Set the flight mode of the drone from the receiver 
  receiver.setFlightMode(&drone);

  // Run the drone feedback-loop
  drone.run();
}
