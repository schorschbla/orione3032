#pragma once

#include <stdint.h>
#include <driver/pulse_cnt.h>

class PulseCounter
{
public:
    PulseCounter(uint8_t pin);
    ~PulseCounter();

    void begin();
    void end();

    uint32_t ticks() const;
    void reset();

private:
    uint8_t pin;
    uint32_t _ticks;
    unsigned long lastInterruptTime;
    
    void onInterrupt();
    static void onInterruptArg(void *arg);
};