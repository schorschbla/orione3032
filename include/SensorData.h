#pragma once

#include <Arduino.h>

template <typename T>
class MeasuredValue
{
public:
    MeasuredValue(T value, uint32_t timestampMs) : _value(value), _timestampMs(timestampMs) {}
    MeasuredValue(T value) : MeasuredValue(value, millis()) {}
    MeasuredValue() : _timestampMs(0) {}

    MeasuredValue &operator=(const MeasuredValue &other)
    {
        _value = other._value;
        _timestampMs = other._timestampMs;
        return *this;
    }

    MeasuredValue &operator=(T value)
    {
        _value = value;
        _timestampMs = millis();
        return *this;
    }

    T value() const
    {
        return _value;
    }

    uint32_t timestampMs()
    {
        return _timestampMs;
    }

private:
    T _value;
    uint32_t _timestampMs;
};

class SensorData
{
public:
    virtual const MeasuredValue<double> &flowVolumeMl() = 0;
    virtual const MeasuredValue<double> &pressureBar() = 0;
    virtual const MeasuredValue<double> &boilerTemperatureCelsius() = 0;
    virtual const MeasuredValue<double> &brewingUnitTemperatureCelsius() = 0;
    virtual const MeasuredValue<double> &weightGramm() = 0;
};
