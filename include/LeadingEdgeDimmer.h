#pragma once

#include "ZeroCrossDetector.h"

class LeadingEdgeDimmer : private ZeroCrossListener
{
public:
    LeadingEdgeDimmer(uint8_t triacPin, ZeroCrossDetector &zeroCrossDetector);
    ~LeadingEdgeDimmer();

    void begin();
    void end();

    void setLevel(uint8_t level);

private:
    ZeroCrossDetector &zeroCrossDetector;
    uint32_t leadingEdgeDelay;
    uint8_t pin;
    hw_timer_t *timer;

    IRAM_ATTR virtual void onZeroCross();
    IRAM_ATTR void onTimerInterrupt();
    IRAM_ATTR friend void onTimerInterruptArg(void *arg);
};