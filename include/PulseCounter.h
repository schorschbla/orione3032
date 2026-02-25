#pragma once

#include <stdint.h>
#include <driver/pulse_cnt.h>

class PulseCounter
{
public:
    PulseCounter(uint8_t pin);
    ~PulseCounter();

    int begin();
    void end();

    uint32_t ticks();
    void reset();

private:
    uint8_t pin;
    pcnt_unit_handle_t pcntUnit;
    pcnt_channel_handle_t pcntChannel;
    
    void onInterrupt();
    static void onInterruptArg(void *arg);
};