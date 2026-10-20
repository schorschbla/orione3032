#pragma once

#include <Arduino.h>

#include <Adafruit_MAX31865.h>
#include <Adafruit_VL53L0X.h>

#include "HardwareConfig.h"
#include "Config.h"
#include "Gc9a01Display.h"
#include "SolidStateRelay.h"
#include "LeadingEdgeDimmer.h"
#include "Xdb401PressureSensor.h"
#include "Mlx90614TemperatureSensor.h"
#include "BleServer.h"
#include "PulseCounter.h"
#include "Actuators.h"
#include "SensorData.h"

#include "StandbyMode.h"


class Qm3032 : private Actuators, private SensorData
{
public:
    Qm3032(const HardwareConfig &hardwareConfig, const ConfigValues &configDefaults);

    void setup();
    void loop();

private:
    Config config;
    HardwareConfig hardwareConfig;
    Gc9a01Display display;
    AcZeroCrossDetector zeroCrossDetector;
    SolidStateRelay heatingRelay;
    LeadingEdgeDimmer pumpDimmer;
    Xdb401PressureSensor pressureSensor;
    Mlx90614TemperatureSensor brewingUnitThermometer;
    SPIClass hspi;
    Adafruit_MAX31865 boilerThermometer;
    Adafruit_VL53L0X waterLevelSensor;
    PulseCounter flowMeter;
    BleServer bleServer;
    bool waterLevelSensorPresent;
    Mode &currentMode;
    Ui &currentUi;

    StandbyMode standbyMode;
    Mode *modes[1] = { &standbyMode };

    uint32_t cycle;
    bool _valveClosed;

    SensorValue<double, 25> _pressureBar;
    SensorValue<double, 20> _boilerTemperatureCelsius;
    SensorValue<double, 20> _brewingUnitTemperatureCelsius;
    SensorValue<double, 8> _weightGramm;
    SensorValue<double, 8> _flowVolumeMl;
    SensorValue<uint32_t, 9> _acHalfWaveLengthUs;

    TaskHandle_t uiTaskHandle;
    void uiThread();
    static void uiTask(void *context);

    void setHeatingPowerAcHalfWaveCount(uint32_t cycles, bool reset = true) override;
    void setValveClosed(bool closed) override;
    bool valveClosed() override;
    void setPumpPowerLevel(float fract) override;
    float pumpPowerLevel() override;

    const SensorValue<double, 8> &flowVolumeMl() override;
    const SensorValue<double, 25> &pressureBar() override;
    const SensorValue<double, 20> &boilerTemperatureCelsius() override;
    const SensorValue<double, 20> &brewingUnitTemperatureCelsius() override;
    const SensorValue<double, 8> &weightGramm() override;
    const SensorValue<uint32_t, 9> &acHalfWaveLengthUs() override;
};   
