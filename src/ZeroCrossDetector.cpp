# include "ZeroCrossDetector.h"

const uint32_t ZeroCrossThresholdUs = 9000;

ZeroCrossDetector::ZeroCrossDetector(uint8_t pin) : pin(pin), lastZeroCrossTime(0), listeners({0})
{
}

ZeroCrossDetector::~ZeroCrossDetector()
{
    end();
}

void onInterruptArg(void *arg)
{
    static_cast<ZeroCrossDetector*>(arg)->onInterrupt();
}

void ZeroCrossDetector::begin()
{
	pinMode(pin, INPUT_PULLDOWN);
	attachInterruptArg(pin, onInterruptArg, this, RISING);
}

void ZeroCrossDetector::end()
{
	detachInterrupt(pin);
}

bool ZeroCrossDetector::addListener(ZeroCrossListener *listener)
{
    int slot = -1;
    for (int i = 0; i < sizeof(listeners) / sizeof(ZeroCrossListener*); ++i)
    {
        if (listeners[i] == listener)
        {
            return false;
        }
        else if (slot < 0 && listeners[i] == nullptr)
        {
            slot = i;
        }
    }
    if (slot < 0)
    {
        return false;
    }
    listeners[slot] = listener;
    return true;
}

bool ZeroCrossDetector::removeListener(ZeroCrossListener *listener)
{
    for (int i = 0; i < sizeof(listeners) / sizeof(ZeroCrossListener*); ++i)
    {
        if (listeners[i] == listener)
        {
            listeners[i] = nullptr;
            return true;
        }
    }
    return false;
}

void ZeroCrossDetector::onInterrupt()
{
    uint32_t time = micros();
	if (lastZeroCrossTime == 0 || time - lastZeroCrossTime > ZeroCrossThresholdUs)
    {
        for (int i = 0; i < sizeof(listeners) / sizeof(ZeroCrossListener*); ++i)
        {
            if (listeners[i] != nullptr)
            {
                listeners[i]->onZeroCross();
            }
        }
        lastZeroCrossTime = time;
    }
}
