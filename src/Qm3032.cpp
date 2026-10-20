#include "Qm3032.h"

#include <Wire.h>

const uint32_t CycleLengthMs = 40;

Qm3032::Qm3032(const HardwareConfig &hardwareConfig, const ConfigValues &configDefaults)
    : hardwareConfig(hardwareConfig),
      config(configDefaults),
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
      brewingUnitThermometer(Wire),
      hspi(HSPI),
      boilerThermometer(hardwareConfig.Max31865PinCs, &hspi),
      waterLevelSensor(Adafruit_VL53L0X()),
      flowMeter(hardwareConfig.PinFlowMeter),
      bleServer(),
      cycle(0),
      uiTaskHandle(nullptr),
      waterLevelSensorPresent(false),
      _valveClosed(false),
      currentMode(standbyMode),
      currentUi(currentMode),
      standbyMode(config)
{
}

static bool probeDevice(TwoWire &wire, uint8_t address)
{
    wire.beginTransmission(address);
    return wire.endTransmission() == 0;
}

void Qm3032::setup()
{
    Serial.begin(115200);

    analogWriteFrequency(hardwareConfig.Gc9a01PinBl, hardwareConfig.Gc9a01BlPwmFrequency);
    analogWrite(hardwareConfig.Gc9a01PinBl, 0);

    xTaskCreatePinnedToCore(uiTask, "uiTask", 10000, NULL, 1, &uiTaskHandle, 0);

    heatingRelay.begin();
    pumpDimmer.begin();
    zeroCrossDetector.begin();
    pinMode(hardwareConfig.AcPinValve, OUTPUT);

    Wire.begin();

    waterLevelSensorPresent = probeDevice(Wire, VL53L0X_I2C_ADDR);
    if (waterLevelSensorPresent) 
    {
        waterLevelSensor.begin();
        waterLevelSensor.configSensor(Adafruit_VL53L0X::VL53L0X_Sense_config_t::VL53L0X_SENSE_HIGH_ACCURACY);
        waterLevelSensor.startRange();
    }

    hspi.begin(hardwareConfig.Max31865PinSclk, hardwareConfig.Max31865PinMiso, hardwareConfig.Max31865PinMosi);

    boilerThermometer.begin(MAX31865_3WIRE);
    boilerThermometer.enable50Hz(true);
    boilerThermometer.autoConvert(true);
    boilerThermometer.enableBias(true);

    flowMeter.begin();

    bleServer.start("SchorschCoff");

    for (int i = 0; i < sizeof(modes) / sizeof(Mode *); ++i)
    {
        modes[i]->begin(CycleLengthMs);
    }
}

void Qm3032::loop()
{
    uint32_t windowStartTimeMs = millis();

    bool updateUi = eTaskGetState(uiTaskHandle) == eTaskState::eSuspended;

    currentMode.loop(*this, *this, cycle, updateUi);

    if (updateUi)
    {
        currentUi = currentMode;
        vTaskResume(uiTaskHandle);
    }

    if (windowStartTimeMs >= _boilerTemperatureCelsius.sample().timestampMs() + hardwareConfig.Max31856SampleTimeMs)
    {
        uint16_t sampleValue = boilerThermometer.readRTDCont();
        double boilerTemperatureCelsiusSample = boilerThermometer.calculateTemperature(sampleValue,
            hardwareConfig.Max31865ReferenceTemperatureCelsius,
            hardwareConfig.Max31865ReferenceResistorValueOhms);
        _boilerTemperatureCelsius.addSample(SensorSample(boilerTemperatureCelsiusSample));

        double brewingUnitTemperatureSample;
        if (brewingUnitThermometer.readValue(Mlx90614TemperatureSensor::Register::Obj1, brewingUnitTemperatureSample) == 0)
        {
            _brewingUnitTemperatureCelsius.addSample(SensorSample(brewingUnitTemperatureSample));
        }
    }

    double pressureSample;
    if (pressureSensor.readValue(pressureSample) == 0)
    {
        _pressureBar.addSample(SensorSample(pressureSample));
    }

    uint32_t flowMeterTicksSample;
    uint32_t timestamp;
    flowMeter.ticks(flowMeterTicksSample, timestamp);
    if (timestamp != _flowVolumeMl.sample().timestampMs())
    {
        _flowVolumeMl.addSample(SensorSample<double>(flowMeterTicksSample * hardwareConfig.FlowMeterVolumePerTickMl, timestamp));
    }

    int32_t valueTenthGrammsSample;
    bleServer.scaleValue(valueTenthGrammsSample, timestamp);
    if (timestamp != _weightGramm.sample().timestampMs())
    {
        _weightGramm.addSample(SensorSample<double>(valueTenthGrammsSample / 10.0, timestamp));
    }

    uint32_t phaseDurationUsSample;
    unsigned long lastZeroCrossTimeUs;
    zeroCrossDetector.phaseDurationUs(phaseDurationUsSample, lastZeroCrossTimeUs);
    timestamp = lastZeroCrossTimeUs / 1000;
    if (timestamp != _acHalfWaveLengthUs.sample().timestampMs())
    {
        _acHalfWaveLengthUs.addSample(SensorSample<uint32_t>(phaseDurationUsSample, timestamp));
    }

    uint32_t elapsedMs = millis() - windowStartTimeMs;
    if (elapsedMs < CycleLengthMs)
    {
        delay(CycleLengthMs - elapsedMs);
    }

    cycle++;
}

void Qm3032::uiThread()
{
    display.init();
    display.setRotation(hardwareConfig.Gc9a01Rotation);

    lv_init();
    lv_disp_drv_register(&display.lvglDriver());

    for (int i = 0; i < sizeof(modes) / sizeof(Mode *); ++i)
    {
        modes[i]->initUi();
    }

    for (;;)
    {
        vTaskSuspend(NULL);
        lv_scr_load(currentUi.screen());
        lv_timer_handler();
    }
}

void Qm3032::uiTask(void *context)
{
    reinterpret_cast<Qm3032*>(context)->uiThread();
    vTaskDelete(nullptr);
}

void Qm3032::setHeatingPowerAcHalfWaveCount(uint32_t cycles, bool reset)
{
    heatingRelay.setCycles(cycles, reset);
}

void Qm3032::setValveClosed(bool closed)
{
    if (closed != _valveClosed)
    {
        _valveClosed = closed;
        digitalWrite(hardwareConfig.AcPinValve, _valveClosed ? HIGH : LOW);
    }
}

bool Qm3032::valveClosed()
{
    return _valveClosed;
}

void Qm3032::setPumpPowerLevel(float level)
{
    pumpDimmer.setPowerLevel(level);
}

float Qm3032::pumpPowerLevel()
{
    return pumpDimmer.powerLevel();
}

const SensorValue<double, 8> &Qm3032::flowVolumeMl()
{
    return _flowVolumeMl;
}

const SensorValue<double, 25> &Qm3032::pressureBar()
{
    return _pressureBar;
}

const SensorValue<double, 20> &Qm3032::boilerTemperatureCelsius()
{
    return _boilerTemperatureCelsius;
}

const SensorValue<double, 20> &Qm3032::brewingUnitTemperatureCelsius()
{
    return _brewingUnitTemperatureCelsius;
}

const SensorValue<double, 8> &Qm3032::weightGramm()
{
    return _weightGramm;
}

const SensorValue<uint32_t, 9> &Qm3032::acHalfWaveLengthUs() 
{
    return _acHalfWaveLengthUs;
}
