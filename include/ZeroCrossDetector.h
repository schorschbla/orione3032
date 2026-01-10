#pragma once

#include <stdint.h>
#include <Arduino.h>

#define MAX_ZEROCROSS_LISTENERS     8

class ZeroCrossListener
{
public:
    virtual IRAM_ATTR void onZeroCross() = 0;
};

class ZeroCrossDetector
{
public:
    ZeroCrossDetector(uint8_t pin);
    ~ZeroCrossDetector();

    void begin();
    void end();

    bool addListener(ZeroCrossListener *listener);
    bool removeListener(ZeroCrossListener *listener);

private:
    uint8_t pin;
    uint32_t lastZeroCrossTime;
    ZeroCrossListener* listeners[MAX_ZEROCROSS_LISTENERS];

    void onInterrupt();
    friend void onInterruptArg(void *arg);
};