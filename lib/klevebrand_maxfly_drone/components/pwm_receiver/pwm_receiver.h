#ifndef PWM_RECEIVER_H
#define PWM_RECEIVER_H

#include <Arduino.h>
#include <PinChangeInterrupt.h>

#define CHANNEL_COUNT 8

class PwmReceiver
{
public:
    void begin();
    int getChannelValue(int channelNumber);

private:
    static void recordPinChangePulseWidth(int channelNumber);
    static void recordPinChangePulseWidthChannel1() { recordPinChangePulseWidth(1); };
    static void recordPinChangePulseWidthChannel2() { recordPinChangePulseWidth(2); };
    static void recordPinChangePulseWidthChannel3() { recordPinChangePulseWidth(3); };
    static void recordPinChangePulseWidthChannel4() { recordPinChangePulseWidth(4); };
    static void recordPinChangePulseWidthChannel5() { recordPinChangePulseWidth(5); };
    static void recordPinChangePulseWidthChannel6() { recordPinChangePulseWidth(6); };
    static void recordPinChangePulseWidthChannel7() { recordPinChangePulseWidth(7); };
    static void recordPinChangePulseWidthChannel8() { recordPinChangePulseWidth(8); };
    static volatile int channelNumberToGpioMapArray[CHANNEL_COUNT];
    static volatile unsigned long pulseStartMicros[CHANNEL_COUNT];
    static volatile int pulseWidths[CHANNEL_COUNT];
};

#endif
