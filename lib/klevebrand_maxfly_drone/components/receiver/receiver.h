#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>
#include <PinChangeInterrupt.h>

#define CHANNEL_COUNT 8

class Receiver
{
public:
    void begin();
    int getChannelValue(int channelNumber);
private:
    static void handlePinChange();
    static volatile int channelNumberToGpioMapArray[CHANNEL_COUNT];
    static volatile unsigned long pulseStartMicros[CHANNEL_COUNT];
    static volatile int pulseWidths[CHANNEL_COUNT];
};

#endif
