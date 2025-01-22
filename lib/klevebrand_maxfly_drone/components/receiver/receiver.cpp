#include "receiver.h"

int Receiver::getChannelValue(int channelNumber)
{
    if (channelNumber < 1 || channelNumber > CHANNEL_COUNT)
    {
        return -1;
    }
    int index = channelNumber - 1;

    return channelValuesArray[index];
}

void Receiver::begin()
{
    for (int i = 0; i < CHANNEL_COUNT; ++i)
    {
        pinMode(channelNumberToGpioMapArray[i], INPUT);
    }
}

void Receiver::pollChannels()
{
    for (int i = 0; i < CHANNEL_COUNT; ++i)
    {
        int pinState = digitalRead(channelNumberToGpioMapArray[i]);

        if (pinState == HIGH && latestSignalStartTimestampInMillisecondsArray[i] == 0)
        {
            // A new signal has started
            latestSignalStartTimestampInMillisecondsArray[i] = micros();
        }
        else if (pinState == LOW && latestSignalStartTimestampInMillisecondsArray[i] != 0)
        {
            // The signal has ended
            channelValuesArray[i] = (int)(micros() - latestSignalStartTimestampInMillisecondsArray[i]);
            latestSignalStartTimestampInMillisecondsArray[i] = 0;
        }
    }
}