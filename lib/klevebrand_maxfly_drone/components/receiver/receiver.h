#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>

#define CHANNEL_COUNT 8

class Receiver
{
public:
    Receiver(int channelOneGpioPortNumber,
             int channelTwoGpioPortNumber,
             int channelThreeGpioPortNumber,
             int channelFourGpioPortNumber,
             int channelFiveGpioPortNumber,
             int channelSixGpioPortNumber,
             int channelSevenGpioPortNumber,
             int channelEightGpioPortNumber)
    {
        this->channelNumberToGpioMapArray = new volatile int[CHANNEL_COUNT]{
            channelOneGpioPortNumber,
            channelTwoGpioPortNumber,
            channelThreeGpioPortNumber,
            channelFourGpioPortNumber,
            channelFiveGpioPortNumber,
            channelSixGpioPortNumber,
            channelSevenGpioPortNumber,
            channelEightGpioPortNumber};

        this->channelValuesArray = new volatile int[CHANNEL_COUNT]{1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
        this->latestSignalStartTimestampInMillisecondsArray = new volatile unsigned long[CHANNEL_COUNT]{0, 0, 0, 0, 0, 0, 0, 0};
    }

    ~Receiver()
    {
        delete[] channelNumberToGpioMapArray;
        delete[] channelValuesArray;
        delete[] latestSignalStartTimestampInMillisecondsArray;
    }

    void begin();
    int getChannelValue(int channelNumber);
    void pollChannels();

private:
    volatile int *channelNumberToGpioMapArray;
    volatile int *channelValuesArray;
    volatile unsigned long *latestSignalStartTimestampInMillisecondsArray;
};

#endif
