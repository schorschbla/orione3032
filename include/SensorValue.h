#pragma once

#include <Arduino.h>
#include <algorithm>

template <typename T>
class SensorSample
{
public:
    SensorSample(T value, uint32_t timestampMs) : _value(value), _timestampMs(timestampMs) {}
    SensorSample(T value) : SensorSample(value, millis()) {}
    SensorSample() : _timestampMs(0) {}

    SensorSample &operator=(const SensorSample &other)
    {
        _value = other._value;
        _timestampMs = other._timestampMs;
        return *this;
    }

    SensorSample &operator=(T value)
    {
        _value = value;
        _timestampMs = millis();
        return *this;
    }

    T value() const
    {
        return _value;
    }

    uint32_t timestampMs() const
    {
        return _timestampMs;
    }

private:
    T _value;
    uint32_t _timestampMs;
};

template <typename T, uint32_t SAMPLECOUNT, typename SUMTYPE = T>
class SensorValue
{
public:
    SensorValue() : pos(SAMPLECOUNT - 1)
    {
    }

    const SensorSample<T> &sample(uint32_t backwardIndex = 0) const
    {
        return samples[(pos + SAMPLECOUNT - backwardIndex % SAMPLECOUNT) % SAMPLECOUNT];
    }

    void addSample(const SensorSample<T> &sample)
    {
        pos = (pos + 1) % SAMPLECOUNT;
        samples[pos] = sample;
    }

    T average(uint32_t durationMs = UINT_MAX) const
    {
        const uint32_t now = millis();
        SUMTYPE sum = 0;
        uint32_t count = 0;

        for (uint32_t i = 0; i < SAMPLECOUNT; ++i)
        {
            const SensorSample<T> &current = sample(i);
            if (current.timestampMs() != 0 && now - current.timestampMs() <= durationMs)
            {
                sum += static_cast<SUMTYPE>(current.value());
                ++count;
            }
        }

        return count == 0 ? T{} : static_cast<T>(sum / static_cast<SUMTYPE>(count));
    }

    T median(uint32_t durationMs = UINT_MAX) const
    {
        const uint32_t now = millis();
        SensorSample<T> values[SAMPLECOUNT];
        uint32_t count = 0;

        for (uint32_t i = 0; i < SAMPLECOUNT; ++i)
        {
            const SensorSample<T> &current = sample(i);
            if (current.timestampMs() != 0 && now - current.timestampMs() <= durationMs)
            {
                values[count++] = current;
            }
        }

        if (count == 0)
        {
            return 0;
        }

        std::sort(values, values + count, [](const SensorSample<T> &left, const SensorSample<T> &right)
        { 
            return left.value() < right.value();
        });

        const uint32_t middle = count / 2;
        if (count % 2 == 0)
        {
            return static_cast<SUMTYPE>(values[middle - 1].value()) +
                static_cast<SUMTYPE>(values[middle].value()) / 2;
        }
        return values[middle].value();
    }

private:
    SensorSample<T> samples[SAMPLECOUNT];
    uint32_t pos;
};