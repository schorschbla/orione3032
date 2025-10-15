#include "LeadingEdgeDimmer.h"

const uint32_t CycleLength = 10000;

LeadingEdgeDimmer::LeadingEdgeDimmer(uint8_t pin, ZeroCrossDetector &zeroCrossDetector) : 
    pin(pin), zeroCrossDetector(zeroCrossDetector), leadingEdgeDelay(CycleLength), timer(nullptr)
{
}

LeadingEdgeDimmer::~LeadingEdgeDimmer()
{
    end();
}

IRAM_ATTR void onTimerInterruptArg(void *arg)
{
    static_cast<LeadingEdgeDimmer*>(arg)->onTimerInterrupt();
}

void LeadingEdgeDimmer::begin()
{
  	pinMode(pin, OUTPUT);

	timer = timerBegin(10000);
	timerStop(timer);
	timerAttachInterruptArg(timer, &onTimerInterruptArg, this);

    zeroCrossDetector.addListener(this);
}

void LeadingEdgeDimmer::end()
{
    zeroCrossDetector.removeListener(this);

    if (timer != nullptr)
    {
        timerEnd(timer);
        timer = nullptr;
    }
}

void LeadingEdgeDimmer::setLevel(uint8_t level)
{
    leadingEdgeDelay = sin((double)(UINT8_MAX - level) / UINT8_MAX * PI / 2) * CycleLength;
}

IRAM_ATTR void LeadingEdgeDimmer::onZeroCross()
{
    if (leadingEdgeDelay != 0)
    {
        digitalWrite(pin, LOW);
        if (leadingEdgeDelay < CycleLength)
        {
            timerWrite(timer, leadingEdgeDelay);
            timerRestart(timer);
        }
    }
    else
    {
        digitalWrite(PIN_PUMP_AC, HIGH);
    }
}

IRAM_ATTR void LeadingEdgeDimmer::onTimerInterrupt()
{
    digitalWrite(pin, HIGH);
}