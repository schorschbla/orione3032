#include <Arduino.h>
#include <driver/gpio.h>
#include "PulseCounter.h"

PulseCounter::PulseCounter(uint8_t pin, uint32_t debouncePeriodMs, int edgeType)
    : pin(pin), debouncePeriodMs(debouncePeriodMs), edgeType(edgeType), _ticks(0), timer(nullptr)
{
}

PulseCounter::~PulseCounter()
{
    end();
}

void PulseCounter::begin()
{
    timer = timerBegin(1000000);
    timerAttachInterruptArg(timer, onTimerInterruptArg, this);
    attachInterruptArg(pin, onInterruptArg, this, edgeType);
}

void PulseCounter::end()
{
    detachInterrupt(pin);

    if (timer != nullptr)
    {
        timerEnd(timer);
        timer = nullptr;
    }
}

uint32_t PulseCounter::ticks() const
{
    return this->_ticks;
}

void PulseCounter::reset()
{
    this->_ticks = 0;
}

IRAM_ATTR void PulseCounter::onInterrupt()
{
    gpio_intr_disable(static_cast<gpio_num_t>(pin));
    this->_ticks++;
    timerRestart(timer);
    timerAlarm(timer, static_cast<uint64_t>(debouncePeriodMs) * 1000, false, 0);
}

IRAM_ATTR void PulseCounter::onTimerInterrupt()
{
    gpio_intr_enable(static_cast<gpio_num_t>(pin));
}

void PulseCounter::onInterruptArg(void *arg)
{
    static_cast<PulseCounter *>(arg)->onInterrupt();
}

void PulseCounter::onTimerInterruptArg(void *arg)
{
    static_cast<PulseCounter *>(arg)->onTimerInterrupt();
}