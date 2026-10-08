#pragma once

#include <stdint.h>
#include <Arduino.h>

#define MAX_ZEROCROSS_LISTENERS     8

class AcZeroCrossListener
{
public:
    virtual IRAM_ATTR void onZeroCross() = 0;
};

class AcZeroCrossDetector
{
public:
    AcZeroCrossDetector(uint8_t pin);
    ~AcZeroCrossDetector();
unsigned int count;
    void begin();
    void end();

    bool addListener(AcZeroCrossListener *listener);
    bool removeListener(AcZeroCrossListener *listener);

    uint32_t phaseLengthUs() const;

private:
    uint8_t pin;
    uint32_t lastZeroCrossTimeUs;
    uint32_t phaseLengthsUs[32];
    uint8_t phaseLengthIndex;
    uint32_t phaseLengthAverageUs;
    AcZeroCrossListener* listeners[MAX_ZEROCROSS_LISTENERS];

    IRAM_ATTR void onInterrupt();
    IRAM_ATTR static void onInterruptArg(void *arg);
};