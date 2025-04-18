#include "../lib/klevebrand_maxfly_drone/klevebrand_maxfly_drone.h"
#include "../lib/klevebrand_maxfly_drone/components/pwm_receiver/pwm_receiver.h"

Drone drone = Drone(6, 7, 2, 3, 3, 4, 2, 1, 7, 5, 8);
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
  //Serial.print(receiver.getChannelValue(1));
  //Serial.print(",");
  //Serial.print(receiver.getChannelValue(2));
  //Serial.print(",");
  //Serial.print(receiver.getChannelValue(3));
  //Serial.print(",");
  //Serial.print(receiver.getChannelValue(4));
  //Serial.print(",");
  //Serial.print(receiver.getChannelValue(5));
  //Serial.print(",");
  //Serial.print(receiver.getChannelValue(6));
  //Serial.print(",");
  //Serial.print(receiver.getChannelValue(7));
  //Serial.print(",");
  //Serial.println(receiver.getChannelValue(8));

  // Run the drone feedback-loop
  drone.run();
}