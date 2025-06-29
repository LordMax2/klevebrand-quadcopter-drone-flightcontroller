#include "drone_pwm_receiver.h"

void DronePwmReceiver::setThrottleYawPitchRoll(Drone *drone)
{
    float throttle_value = receiver.getChannelValue(throttle_receiver_channel_number);
    if (throttle_value >= 1800)
        throttle_value = 2000;
    drone->setThrottle(throttle_value);

    // Currently disabled unitl successful hover flight
    // setDesiredYawAngle(receiver.getChannelValue(yawReceiverChannelNumber));

    float desired_pitch_angle = map(receiver.getChannelValue(pitch_receiver_channel_number), 1060, 1900, -40, 40) * -1;
    drone->setDesiredPitchAngle(desired_pitch_angle);

    float desired_roll_angle = map(receiver.getChannelValue(roll_receiver_channel_number), 1060, 1900, -40, 40);
    drone->setDesiredRollAngle(desired_roll_angle);
}

void DronePwmReceiver::setPid(Drone *drone)
{
    float pid_p_value = mapfloat(receiver.getChannelValue(pid_p_constant_channel_number), 1100, 1800, 0, 10);
    drone->setPidPConstant(pid_p_value / 5);

    float pid_i_value = mapfloat(receiver.getChannelValue(pid_i_constant_channel_number), 1100, 1800, 0, 1) / 10;
    drone->setPidIConstant(pid_i_value / 5);

    float pid_d_value = mapfloat(receiver.getChannelValue(pid_d_constant_channel_number), 1100, 1800, 0, 60);
    drone->setPidDConstant(pid_d_value / 5);
}

void DronePwmReceiver::setup()
{
    receiver.setup();
}
