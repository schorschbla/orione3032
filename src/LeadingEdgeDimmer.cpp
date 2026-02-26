#include "LeadingEdgeDimmer.h"

const uint32_t MicrosPerSecond = 1000000;
const uint32_t CycleLengthMicros = 10000;

LeadingEdgeDimmer::LeadingEdgeDimmer(uint8_t pin, AcZeroCrossDetector &zeroCrossDetector) : 
    pin(pin), zeroCrossDetector(zeroCrossDetector), leadingEdgeLengthMicros(CycleLengthMicros), timer(nullptr)
{
}

LeadingEdgeDimmer::~LeadingEdgeDimmer()
{
    end();
}

void LeadingEdgeDimmer::begin()
{
  	pinMode(pin, OUTPUT);

	timer = timerBegin(MicrosPerSecond);
	timerAttachInterruptArg(timer, &LeadingEdgeDimmer::onTimerInterruptArg, this);

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

void LeadingEdgeDimmer::setPowerLevel(double level)
{
    level = constrain(level, 0.0, 1.0);
    leadingEdgeLengthMicros = acos(2.0 * level - 1.0) / PI * CycleLengthMicros;
}

void LeadingEdgeDimmer::onZeroCross()
{
    if (leadingEdgeLengthMicros != 0)
    {
        digitalWrite(pin, LOW);
        if (leadingEdgeLengthMicros < CycleLengthMicros)
        {
            timerRestart(timer);
            timerAlarm(timer, leadingEdgeLengthMicros, false, 0);
        }
    }
    else
    {
        digitalWrite(pin, HIGH);
    }
}

void LeadingEdgeDimmer::onTimerInterrupt()
{
    digitalWrite(pin, HIGH);
}

void LeadingEdgeDimmer::onTimerInterruptArg(void *arg)
{
    static_cast<LeadingEdgeDimmer*>(arg)->onTimerInterrupt();
}