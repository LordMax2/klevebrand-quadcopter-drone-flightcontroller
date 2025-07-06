#ifndef PWM_RECEIVER_H
#define PWM_RECEIVER_H

#include <Arduino.h>
#include <PinChangeInterrupt.h>

#define CHANNEL_COUNT 8

class PwmReceiver
{
public:
    void setup();
    int getChannelValue(int channel_number);

private:
    static void recordPinChangePulseWidth(int channel_number);
    static void recordPinChangePulseWidthChannel1() { recordPinChangePulseWidth(1); };
    static void recordPinChangePulseWidthChannel2() { recordPinChangePulseWidth(2); };
    static void recordPinChangePulseWidthChannel3() { recordPinChangePulseWidth(3); };
    static void recordPinChangePulseWidthChannel4() { recordPinChangePulseWidth(4); };
    static void recordPinChangePulseWidthChannel5() { recordPinChangePulseWidth(5); };
    static void recordPinChangePulseWidthChannel6() { recordPinChangePulseWidth(6); };
    static void recordPinChangePulseWidthChannel7() { recordPinChangePulseWidth(7); };
    static void recordPinChangePulseWidthChannel8() { recordPinChangePulseWidth(8); };
    static volatile int channel_number_to_gpio_map_array[CHANNEL_COUNT];
    static volatile unsigned long pulse_start_micros[CHANNEL_COUNT];
    static volatile int pulse_widths[CHANNEL_COUNT];
};

#endif
