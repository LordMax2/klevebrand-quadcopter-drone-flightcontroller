#ifndef DRONE_PWM_RECEIVER_H
#define DRONE_PWM_RECEIVER_H

#include "../pwm_receiver/pwm_receiver.h"
#include "../../klevebrand_maxfly_drone.h"

class DronePwmReceiver
{
public:
    DronePwmReceiver(
        int throttle_receiver_channel_number,
        int yaw_receiver_channel_number,
        int pitch_receiver_channel_number,
        int roll_receiver_channel_number,
        int pid_p_constant_channel_number,
        int pid_i_constant_channel_number,
        int pid_d_constant_channel_number)
    {
        /*
         * Map the reciever PID channel numbers
         */
        this->pid_p_constant_channel_number = pid_p_constant_channel_number;
        this->pid_i_constant_channel_number = pid_i_constant_channel_number;
        this->pid_d_constant_channel_number = pid_d_constant_channel_number;

        /*
         * Map the receiver channel numbers
         */
        this->throttle_receiver_channel_number = throttle_receiver_channel_number;
        this->yaw_receiver_channel_number = yaw_receiver_channel_number;
        this->pitch_receiver_channel_number = pitch_receiver_channel_number;
        this->roll_receiver_channel_number = roll_receiver_channel_number;
    };

    void setup();
    void setThrottleYawPitchRoll(Drone *drone);
    void setPid(Drone *drone);

private:
    PwmReceiver receiver;

    int throttle_receiver_channel_number;
    int yaw_receiver_channel_number;
    int pitch_receiver_channel_number;
    int roll_receiver_channel_number;
    int pid_p_constant_channel_number;
    int pid_i_constant_channel_number;
    int pid_d_constant_channel_number;

    float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

#endif // DRONE_PWM_RECEIVER_H