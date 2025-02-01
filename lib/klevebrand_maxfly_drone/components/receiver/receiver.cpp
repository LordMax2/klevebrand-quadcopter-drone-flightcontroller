#include "receiver.h"

volatile int Receiver::channelNumberToGpioMapArray[CHANNEL_COUNT] = {8, 61, 62, 63, 64, 65, 66, 78};
volatile unsigned long Receiver::pulseStartMicros[CHANNEL_COUNT] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
volatile int Receiver::pulseWidths[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};

void Receiver::begin()
{
    for (int i = 0; i < CHANNEL_COUNT; i++) {
        pinMode(channelNumberToGpioMapArray[i], INPUT);
        delay(1000);
        attachPCINT(digitalPinToPCINT(channelNumberToGpioMapArray[i]), handlePinChange, CHANGE);
    }
}

void Receiver::handlePinChange()
{
    for (int i = 0; i < CHANNEL_COUNT; i++) {
        int pinState = digitalRead(channelNumberToGpioMapArray[i]);
        if (pinState == HIGH) {
            pulseStartMicros[i] = micros();
        } else {
            if (pulseStartMicros[i] != 0) {
                pulseWidths[i] = (int)(micros() - pulseStartMicros[i]);
                pulseStartMicros[i] = 0;
            }
        }
    }
}

int Receiver::getChannelValue(int channelNumber)
{
    if (channelNumber < 1 || channelNumber > CHANNEL_COUNT) {
        return -1;
    }
    return pulseWidths[channelNumber - 1];
}