#include "Qm3032.h"

static struct HardwareConfig hardwareConfig = 
{
    .Gc9a01PinMosi = 14,
    .Gc9a01PinSclk = 26,
    .Gc9a01PinCs = 12,
    .Gc9a01PinDc = 27,
    .Gc9a01PinRst = 13,
    .Gc9a01PinBl = 2,
    .Gc9a01Frequency = 70000000,
    .Gc9a01BlPwmFrequency = 2000,
    .Gc9a01Rotation = 1,

    .AcPinZeroCross = 18,
    .AcPinPump = 17,
    .AcPinValve = 16,
    .AcPinHeating = 4,

    .Max31865PinMiso = 35,
    .Max31865PinMosi = 25,
    .Max31865PinSclk = 33,
    .Max31865PinCs = 32,
    .Max31856SampleTimeMs = 80,
    .Max31865ReferenceTemperatureCelsius = 100.0,
    .Max31865ReferenceResistorValueOhms = 430.0,

    .PinSwitchInfuse = 34,
    .PinSwitchSteam = 23,
    //.PinHotwaterSwitch = 39,

    .PinFlowMeter = 19,
    .PinBuzzer = 5,

    .Xdb401MaxBar = 20.0,

    .FlowMeterVolumePerTickMl = 0.23
};

Qm3032 qm3032(hardwareConfig);

void setup()
{
    qm3032.setup();
}

void loop()
{
    qm3032.loop();
}