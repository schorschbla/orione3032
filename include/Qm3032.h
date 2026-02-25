#pragma once

#include "Gradient.h"
#include "Gc9a01Display.h"
#include "SolidStateRelay.h"
#include "LeadingEdgeDimmer.h"
#include "Xdb401PressureSensor.h"
#include "Mlx90614TemperatureSensor.h"

typedef struct
{
    uint8_t Gc9a01PinMosi;
    uint8_t Gc9a01PinSclk;
    uint8_t Gc9a01PinCs;
    uint8_t Gc9a01PinDc;
    uint8_t Gc9a01PinRst;
    uint8_t Gc9a01PinBl;
    uint32_t Gc9a01Frequency;

    uint8_t AcPinZeroCross;
    uint8_t AcPinPump;
    uint8_t AcPinValve;
    uint8_t AcPinHeating;

    uint8_t Max31865PinMiso;
    uint8_t Max31865PinMosi;
    uint8_t Max31865PinSclk;
    uint8_t Max31865PinCs;

    uint8_t PinSwitchInfuse;
    uint8_t PinSwitchSteam;

    uint8_t PinFlowMeterCold;
    uint8_t PinBuzzer;
} Qm3033HardwareConfig;

class Qm3032
{
public:
    Qm3032(const Qm3033HardwareConfig &hardwareConfig);

private:
    const Qm3033HardwareConfig &hardwareConfig;
    Gc9a01Display display;
    IRAM_ATTR AcZeroCrossDetector zeroCrossDetector;
    IRAM_ATTR SolidStateRelay heatingRelay;
    IRAM_ATTR LeadingEdgeDimmer pumpDimmer;
    Xdb401PressureSensor pressureSensor;
    Mlx90614TemperatureSensor brewingUnitTemperatureSensor;
};