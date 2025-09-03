#include "drone_pwm_receiver.h"

void DronePwmReceiver::setThrottleYawPitchRoll(Drone *drone)
{
    float throttle_value = receiver.getChannelValue(throttle_receiver_channel_number);
    drone->setThrottle(throttle_value);

    if (drone->getFlightMode() == acro)
    {
        float desired_yaw_angle = map(receiver.getChannelValue(yaw_receiver_channel_number), 1000, 2000, 180, -180);
        if (desired_yaw_angle < 5 && desired_yaw_angle > -5)
        {
            desired_yaw_angle = 0;
        }
        drone->setDesiredYawAngle(desired_yaw_angle);

        float desired_pitch_angle = map(receiver.getChannelValue(pitch_receiver_channel_number), 1000, 2000, -120, 120);
        if (desired_pitch_angle < 5 && desired_pitch_angle > -5)
        {
            desired_pitch_angle = 0;
        }
        drone->setDesiredPitchAngle(desired_pitch_angle);

        float desired_roll_angle = map(receiver.getChannelValue(roll_receiver_channel_number), 1000, 2000, -120, 120);
        if (desired_roll_angle < 5 && desired_roll_angle > -5)
        {
            desired_roll_angle = 0;
        }
        drone->setDesiredRollAngle(desired_roll_angle);

        return;
    }

    if (drone->getFlightMode() == auto_level)
    {
        float desired_yaw_angle = map(receiver.getChannelValue(yaw_receiver_channel_number), 1000, 2000, -60, 60);
        drone->setDesiredYawAngle(desired_yaw_angle);

        float desired_pitch_angle = map(receiver.getChannelValue(pitch_receiver_channel_number), 1000, 2000, -60, 60) * -1;
        drone->setDesiredPitchAngle(desired_pitch_angle);

        float desired_roll_angle = map(receiver.getChannelValue(roll_receiver_channel_number), 1000, 2000, -60, 60);
        drone->setDesiredRollAngle(desired_roll_angle);

        return;
    }
}

void DronePwmReceiver::setFlightMode(Drone *drone) 
{
    int flight_mode_pwm_signal = receiver.getChannelValue(flight_mode_receiver_channel_number);

    switch(flight_mode_pwm_signal) {
        case 1000:
            drone->disableMotors();
        case 1500: 
            drone->enableMotors();
            drone->setFlightModeAutoLevel();
        case 2000: 
            drone->enableMotors();
            drone->setFlightModeAcro();
        break;
    }
}

void DronePwmReceiver::setup()
{
    receiver.setup();
}
