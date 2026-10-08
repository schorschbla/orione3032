#pragma once

#include "HardwareConfig.h"
#include "Gc9a01Display.h"
#include "SolidStateRelay.h"
#include "LeadingEdgeDimmer.h"
#include "Xdb401PressureSensor.h"
#include "Mlx90614TemperatureSensor.h"
#include "BleServer.h"
#include "PulseCounter.h"
#include "Actuators.h"
#include "SensorData.h"

class Qm3032 : private Actuators, private SensorData
{
public:
    Qm3032(const HardwareConfig &hardwareConfig);

    void setup();
    void loop();

private:
    HardwareConfig hardwareConfig;
    Gc9a01Display display;
    AcZeroCrossDetector zeroCrossDetector;
    SolidStateRelay heatingRelay;
    LeadingEdgeDimmer pumpDimmer;
    Xdb401PressureSensor pressureSensor;
    Mlx90614TemperatureSensor brewingUnitTemperatureSensor;
    PulseCounter flowMeter;
    BleServer bleServer;

    uint32_t cycle;
    
    bool _valveClosed;
    float _pressureBar;
    float _boilerTemperatureCelsius;
    float _brewingUnitTemperatureCelsius;

    void setHeatingPowerCycles(uint32_t cycles) override;
    uint32_t heatingPowerCycleLengthUs() const override;
    void setValveClosed(bool closed) override;
    bool valveClosed() const override;
    void setPumpPowerLevel(float fract) override;
    float pumpPowerLevel() const override;

    void flowVolumeMl(float &volumeMs, uint32_t &timestamp) const override;
    float pressureBar() const override;
    float boilerTemperatureCelsius() const override;
    float brewingUnitTemperatureCelsius() const override;
    void weightGramm(float &weightGramms, uint32_t &timestamp) const override;
};