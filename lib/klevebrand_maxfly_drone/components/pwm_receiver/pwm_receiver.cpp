#include "pwm_receiver.h"

volatile int PwmReceiver::channelNumberToGpioMapArray[CHANNEL_COUNT] = {-1, -1, -1, -1, -1, -1, -1, -1};
volatile unsigned long PwmReceiver::pulseStartMicros[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile int PwmReceiver::pulseWidths[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};

void PwmReceiver::begin()
{
    for (int i = 0; i < CHANNEL_COUNT; i++)
    {
        pinMode(channelNumberToGpioMapArray[i], INPUT);

        // This delay is needed for some reason, otherwise the all of the read values are 0 for some reason.
        delay(1000);

        attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[i]), recordPinChangePulseWidth, CHANGE);
    }
}

void PwmReceiver::recordPinChangePulseWidth()
{
    for (int i = 0; i < CHANNEL_COUNT; i++)
    {
        int pinState = digitalRead(channelNumberToGpioMapArray[i]);

        /*
         * If pin-state is high, stamp down the millis for that pin.
         */
        if (pinState == HIGH)
        {
            pulseStartMicros[i] = micros();
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
        if (pulseStartMicros[i] != 0)
        {
            pulseWidths[i] = (int)(micros() - pulseStartMicros[i]);
            pulseStartMicros[i] = 0;
            return;
        }
    }
}

int PwmReceiver::getChannelValue(int channelNumber)
{
    // Validate the channel-number
    if (channelNumber < 1 || channelNumber > CHANNEL_COUNT)
    {
        return -1;
    }
    return pulseWidths[channelNumber - 1];
}