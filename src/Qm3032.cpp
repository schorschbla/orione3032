#include "Qm3032.h"

#include <Wire.h>

Qm3032::Qm3032(const HardwareConfig &hardwareConfig)
    : hardwareConfig(hardwareConfig),
      display(hardwareConfig.Gc9a01Frequency,
              hardwareConfig.Gc9a01PinSclk,
              hardwareConfig.Gc9a01PinMosi,
              hardwareConfig.Gc9a01PinDc,
              hardwareConfig.Gc9a01PinCs,
              hardwareConfig.Gc9a01PinRst),
      zeroCrossDetector(hardwareConfig.AcPinZeroCross),
      heatingRelay(hardwareConfig.AcPinHeating, zeroCrossDetector),
      pumpDimmer(hardwareConfig.AcPinPump, zeroCrossDetector),
      pressureSensor(Wire, hardwareConfig.Xdb401MaxBar),
      brewingUnitTemperatureSensor(Wire),
      flowMeter(hardwareConfig.PinFlowMeter),
      bleServer(),
      cycle(0),
      _valveClosed(false),
      _pressureBar(0.0),
      _boilerTemperatureCelsius(0.0),
      _brewingUnitTemperatureCelsius(0.0)
{
}

void Qm3032::setup()
{
}

void Qm3032::loop()
{
}

void Qm3032::setHeatingPowerCycles(uint32_t cycles)
{
    heatingRelay.setCycles(cycles);
}

uint32_t Qm3032::heatingPowerCycleLengthUs() const
{
    return zeroCrossDetector.phaseLengthUs();
}

void Qm3032::setValveClosed(bool closed)
{
    digitalWrite(hardwareConfig.AcPinValve, closed ? HIGH : LOW);
}

bool Qm3032::valveClosed() const
{
    return _valveClosed;
}

void Qm3032::setPumpPowerLevel(float level)
{
    pumpDimmer.setPowerLevel(level);
}

float Qm3032::pumpPowerLevel() const
{
    return pumpDimmer.powerLevel();
}

void Qm3032::flowVolumeMl(float &volumeMs, uint32_t &timestamp) const
{
    uint32_t ticks;
    flowMeter.ticks(ticks, timestamp);
    volumeMs = ticks * hardwareConfig.FlowMeterVolumePerTickMl;
}

float Qm3032::pressureBar() const
{
    return _pressureBar;
}

float Qm3032::boilerTemperatureCelsius() const
{
    return _boilerTemperatureCelsius;
}

float Qm3032::brewingUnitTemperatureCelsius() const
{
    return _brewingUnitTemperatureCelsius;
}

void Qm3032::weightGramm(float &weightGramms, uint32_t &timestamp) const
{
    int32_t valueTenthGramms;
    bleServer.scaleValue(valueTenthGramms, timestamp);
    weightGramms = valueTenthGramms / 10.0;
}