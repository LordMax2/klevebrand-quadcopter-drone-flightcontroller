#include "pwm_receiver.h"

volatile int PwmReceiver::channel_number_to_gpio_map_array[CHANNEL_COUNT] = {A8, A9, A10, A11, A12, A13, A14, A15};
volatile unsigned long PwmReceiver::pulse_start_micros[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile int PwmReceiver::pulse_widths[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};

void PwmReceiver::setup()
{
    if (channel_number_to_gpio_map_array[0] != -1)
    {
        pinMode(channel_number_to_gpio_map_array[0], INPUT);
        attachPCINT(digitalPinToPCINT(channel_number_to_gpio_map_array[0]), recordPinChangePulseWidthChannel1, CHANGE);
    }

    if (channel_number_to_gpio_map_array[1] != -1)
    {
        pinMode(channel_number_to_gpio_map_array[1], INPUT);
        attachPCINT(digitalPinToPCINT(channel_number_to_gpio_map_array[1]), recordPinChangePulseWidthChannel2, CHANGE);
    }

    if (channel_number_to_gpio_map_array[2] != -1)
    {
        pinMode(channel_number_to_gpio_map_array[2], INPUT);
        attachPCINT(digitalPinToPCINT(channel_number_to_gpio_map_array[2]), recordPinChangePulseWidthChannel3, CHANGE);
    }

    if (channel_number_to_gpio_map_array[3] != -1)
    {
        pinMode(channel_number_to_gpio_map_array[3], INPUT);
        attachPCINT(digitalPinToPCINT(channel_number_to_gpio_map_array[3]), recordPinChangePulseWidthChannel4, CHANGE);
    }

    if (channel_number_to_gpio_map_array[4] != -1)
    {
        pinMode(channel_number_to_gpio_map_array[4], INPUT);
        attachPCINT(digitalPinToPCINT(channel_number_to_gpio_map_array[4]), recordPinChangePulseWidthChannel5, CHANGE);
    }

    if (channel_number_to_gpio_map_array[5] != -1)
    {
        pinMode(channel_number_to_gpio_map_array[5], INPUT);
        attachPCINT(digitalPinToPCINT(channel_number_to_gpio_map_array[5]), recordPinChangePulseWidthChannel6, CHANGE);
    }

    if (channel_number_to_gpio_map_array[6] != -1)
    {
        pinMode(channel_number_to_gpio_map_array[6], INPUT);
        attachPCINT(digitalPinToPCINT(channel_number_to_gpio_map_array[6]), recordPinChangePulseWidthChannel7, CHANGE);
    }

    if (channel_number_to_gpio_map_array[7] != -1)
    {
        pinMode(channel_number_to_gpio_map_array[7], INPUT);
        attachPCINT(digitalPinToPCINT(channel_number_to_gpio_map_array[7]), recordPinChangePulseWidthChannel8, CHANGE);
    }
}

void PwmReceiver::recordPinChangePulseWidth(int channelNumber)
{
    int channelNumberIndex = channelNumber - 1;

    int pinState = digitalRead(channel_number_to_gpio_map_array[channelNumberIndex]);

    /*
     * If pin-state is high, stamp down the millis for that pin.
     */
    if (pinState == HIGH)
    {
        pulse_start_micros[channelNumberIndex] = micros();

        return;
    }

    /*
     * If the pin state isnt HIGH, it is LOW.
     *
     * First we need to check that we have a currently "high" signal for the pin, if thats the case, we calculate
     * the pulse width with the pulse-start millits timestamp subtracted by the current millis timestamp.
     *
     * Then we reset the pulse-start for this pin.
     */
    if (pulse_start_micros[channelNumberIndex] != 0)
    {
        pulse_widths[channelNumberIndex] = (int)(micros() - pulse_start_micros[channelNumberIndex]);
        pulse_start_micros[channelNumberIndex] = 0;

        return;
    }
}

int PwmReceiver::getChannelValue(int channelNumber)
{
    // Validate the channel-number
    if (channelNumber < 1 || channelNumber > CHANNEL_COUNT)
    {
        return -1;
    }

    return constrain(pulse_widths[channelNumber - 1], 1000, 2000);
}