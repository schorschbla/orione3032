#include "LeadingEdgeDimmer.h"

const uint32_t MicrosPerSecond = 1000000;
const uint32_t CycleLengthMicros = 10000;

LeadingEdgeDimmer::LeadingEdgeDimmer(uint8_t pin, AcZeroCrossDetector &zeroCrossDetector) : 
    pin(pin), zeroCrossDetector(zeroCrossDetector), leadingEdgeDurationUs(CycleLengthMicros), timer(nullptr)
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

void LeadingEdgeDimmer::setPowerLevel(float powerLevel)
{
    this->_powerLevel = constrain(powerLevel, 0.0f, 1.0f);
    leadingEdgeDurationUs = acos(2.0 * this->_powerLevel - 1.0) / PI * zeroCrossDetector.phaseDurationUs();
}

float LeadingEdgeDimmer::powerLevel() const
{
    return _powerLevel;
}

void LeadingEdgeDimmer::onZeroCross()
{
    if (leadingEdgeDurationUs != 0)
    {
        digitalWrite(pin, LOW);
        if (leadingEdgeDurationUs < CycleLengthMicros)
        {
            timerRestart(timer);
            timerAlarm(timer, leadingEdgeDurationUs, false, 0);
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