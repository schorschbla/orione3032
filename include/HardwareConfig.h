#pragma once

#include <Arduino.h>

struct HardwareConfig
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

    uint8_t PinFlowMeter;
    uint8_t PinBuzzer;

    float Xdb401MaxBar;

    float FlowMeterVolumePerTickMl;
};