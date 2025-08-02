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

void DronePwmReceiver::setPid(Drone *drone)
{
    if (drone->getFlightMode() == acro)
    {
        float pid_p_value = mapfloat(receiver.getChannelValue(pid_p_constant_channel_number), 1100, 1800, 0, 10);
        drone->setPidPConstant(pid_p_value / 5);

        float pid_i_value = mapfloat(receiver.getChannelValue(pid_i_constant_channel_number), 1100, 1800, 0, 1) / 10;
        drone->setPidIConstant(pid_i_value / 5);

        float pid_d_value = mapfloat(receiver.getChannelValue(pid_d_constant_channel_number), 1100, 1800, 0, 60);
        drone->setPidDConstant(pid_d_value / 5);

        return;
    }

    if (drone->getFlightMode() == auto_level)
    {
        float pid_p_value = mapfloat(receiver.getChannelValue(pid_p_constant_channel_number), 1100, 1800, 0, 10);
        drone->setPidPConstant(pid_p_value);

        float pid_i_value = mapfloat(receiver.getChannelValue(pid_i_constant_channel_number), 1100, 1800, 0, 1) / 10;
        drone->setPidIConstant(pid_i_value);

        float pid_d_value = mapfloat(receiver.getChannelValue(pid_d_constant_channel_number), 1100, 1800, 0, 60);
        drone->setPidDConstant(pid_d_value);

        return;
    }
}

void DronePwmReceiver::setup()
{
    receiver.setup();
}
