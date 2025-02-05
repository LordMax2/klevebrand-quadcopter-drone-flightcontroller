#include "pwm_receiver.h"

volatile int PwmReceiver::channelNumberToGpioMapArray[CHANNEL_COUNT] = {-1, -1, -1, -1, -1, -1, -1, -1};
volatile unsigned long PwmReceiver::pulseStartMicros[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile int PwmReceiver::pulseWidths[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};

void PwmReceiver::begin()
{
    pinMode(channelNumberToGpioMapArray[0], INPUT);
    attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[0]), recordPinChangePulseWidthChannel1, CHANGE);
    
    pinMode(channelNumberToGpioMapArray[1], INPUT);
    attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[1]), recordPinChangePulseWidthChannel2, CHANGE);
    
    pinMode(channelNumberToGpioMapArray[2], INPUT);
    attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[2]), recordPinChangePulseWidthChannel3, CHANGE);
    
    pinMode(channelNumberToGpioMapArray[3], INPUT);
    attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[3]), recordPinChangePulseWidthChannel4, CHANGE);
    
    pinMode(channelNumberToGpioMapArray[4], INPUT);
    attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[4]), recordPinChangePulseWidthChannel5, CHANGE);
    
    pinMode(channelNumberToGpioMapArray[5], INPUT);
    attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[5]), recordPinChangePulseWidthChannel6, CHANGE);
    
    pinMode(channelNumberToGpioMapArray[6], INPUT);
    attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[6]), recordPinChangePulseWidthChannel7, CHANGE);
    
    pinMode(channelNumberToGpioMapArray[7], INPUT);
    attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[7]), recordPinChangePulseWidthChannel8, CHANGE);
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