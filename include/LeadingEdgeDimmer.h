#pragma once

#include "AcZeroCrossDetector.h"

class LeadingEdgeDimmer : private AcZeroCrossListener
{
public:
    LeadingEdgeDimmer(uint8_t triacPin, AcZeroCrossDetector &zeroCrossDetector);
    ~LeadingEdgeDimmer();

    void begin();
    void end();

    void setLevel(uint8_t level);

private:
    AcZeroCrossDetector &zeroCrossDetector;
    uint32_t leadingEdgeDelay;
    uint8_t pin;
    hw_timer_t *timer;

    virtual void onZeroCross();
    void onTimerInterrupt();
    friend void onTimerInterruptArg(void *arg);
};