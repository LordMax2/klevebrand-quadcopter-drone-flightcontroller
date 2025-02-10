#include "pwm_receiver.h"

volatile int PwmReceiver::channelNumberToGpioMapArray[CHANNEL_COUNT] = {8, 9, -1, -1, -1, -1, -1, -1};
volatile unsigned long PwmReceiver::pulseStartMicros[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile int PwmReceiver::pulseWidths[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};

void PwmReceiver::setup()
{
    if (channelNumberToGpioMapArray[0] != -1)
    {
        pinMode(channelNumberToGpioMapArray[0], INPUT);
        attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[0]), recordPinChangePulseWidthChannel1, CHANGE);
    }

    if (channelNumberToGpioMapArray[1] != -1)
    {
        pinMode(channelNumberToGpioMapArray[1], INPUT);
        attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[1]), recordPinChangePulseWidthChannel2, CHANGE);
    }

    if (channelNumberToGpioMapArray[2] != -1)
    {
        pinMode(channelNumberToGpioMapArray[2], INPUT);
        attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[2]), recordPinChangePulseWidthChannel3, CHANGE);
    }

    if (channelNumberToGpioMapArray[3] != -1)
    {
        pinMode(channelNumberToGpioMapArray[3], INPUT);
        attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[3]), recordPinChangePulseWidthChannel4, CHANGE);
    }

    if (channelNumberToGpioMapArray[4] != -1)
    {
        pinMode(channelNumberToGpioMapArray[4], INPUT);
        attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[4]), recordPinChangePulseWidthChannel5, CHANGE);
    }

    if (channelNumberToGpioMapArray[5] != -1)
    {
        pinMode(channelNumberToGpioMapArray[5], INPUT);
        attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[5]), recordPinChangePulseWidthChannel6, CHANGE);
    }

    if (channelNumberToGpioMapArray[6] != -1)
    {
        pinMode(channelNumberToGpioMapArray[6], INPUT);
        attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[6]), recordPinChangePulseWidthChannel7, CHANGE);
    }

    if (channelNumberToGpioMapArray[7] != -1)
    {
        pinMode(channelNumberToGpioMapArray[7], INPUT);
        attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[7]), recordPinChangePulseWidthChannel8, CHANGE);
    }
}

void PwmReceiver::recordPinChangePulseWidth(int channelNumber)
{
    int channelNumberIndex = channelNumber - 1;

    int pinState = digitalRead(channelNumberToGpioMapArray[channelNumberIndex]);

    /*
     * If pin-state is high, stamp down the millis for that pin.
     */
    if (pinState == HIGH)
    {
        pulseStartMicros[channelNumberIndex] = micros();
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
    if (pulseStartMicros[channelNumberIndex] != 0)
    {
        pulseWidths[channelNumberIndex] = (int)(micros() - pulseStartMicros[channelNumberIndex]);
        pulseStartMicros[channelNumberIndex] = 0;
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
    return pulseWidths[channelNumber - 1];
}