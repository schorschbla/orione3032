#pragma once

#include <Arduino.h>
#include <stdint.h>

class PulseCounter
{
public:
    PulseCounter(uint8_t pin, uint32_t debouncePeriodMs = 40, int edgeType = RISING);
    ~PulseCounter();

    void begin();
    void end();

    uint32_t ticks() const { return this->_ticks; }
    void ticks(uint32_t &ticks, uint32_t &timestamp) const;
    void reset();

private:
    uint8_t pin;
    uint32_t debouncePeriodMs;
    int edgeType;
    uint32_t _ticks;
    uint32_t timestamp;
    hw_timer_t *timer;

    IRAM_ATTR void onInterrupt();
    IRAM_ATTR void onTimerInterrupt();
    static void onInterruptArg(void *arg);
    static void onTimerInterruptArg(void *arg);
};