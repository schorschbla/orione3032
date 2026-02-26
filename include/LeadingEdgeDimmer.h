#pragma once

#include "AcZeroCrossDetector.h"

class LeadingEdgeDimmer : private AcZeroCrossListener
{
public:
    LeadingEdgeDimmer(uint8_t triacPin, AcZeroCrossDetector &zeroCrossDetector);
    ~LeadingEdgeDimmer();

    void begin();
    void end();

    void setPowerLevel(double level);

private:
    AcZeroCrossDetector &zeroCrossDetector;
    uint32_t leadingEdgeLengthMicros;
    uint8_t pin;
    hw_timer_t *timer;

    IRAM_ATTR virtual void onZeroCross();
    IRAM_ATTR void onTimerInterrupt();
    IRAM_ATTR static void onTimerInterruptArg(void *arg);
};