#pragma once

#include <Arduino.h>

const uint8_t PinGc9a01Mosi = 14;
const uint8_t PinGc9a01Sclk = 26;
const uint8_t PinGc9a01Cs = 12;
const uint8_t PinGc9a01Dc = 27;
const uint8_t PinGc9a01Rst = 13;
const uint8_t PinGc9a01Backlight = 2;

const uint8_t PinAcZerocross = 18;

const uint8_t PinPumpAc = 17;
const uint8_t PinHeatingAc = 4;
const uint8_t PinValveAc = 16;

const uint8_t PinMax31865Cs = 32;
const uint8_t PinMax31865Miso = 35;
const uint8_t PinMax31865Mosi = 25;
const uint8_t PinMax31865Sclk = 33;

const uint8_t PinInfuseSwitch = 34;
const uint8_t PinSteamSwitch = 23;
const uint8_t PinHotwaterSwitch = 39;

const uint8_t PinFlowMeter = 15;

const uint8_t PinBuzzer = 5;


const uint32_t Gc9a01SpiWriteFreq = 70000000;

const double Xdb401MaxBar = 20.0;

const double Max31865ReferenceResistorValueOhms = 430.0;
const double Max31865ReferenceTemperature = 100.0;

const double FlowMeterVolumePerTickMilliliters = 0.1;

const double HeatingOutputWatts = 1000;


const double DefaultPidP = 2.6;
const double DefaultPidI = 0.05;
const double DefaultPidD = 30;

const double PidMaxOutput = 100.0;

const double TemperatureSafetyGuardCelsius = 130.0;
const double MaxTemperatureCelsius = 98.0;
const double WarmupTemperatureThresholdKelvin = 6.0;

const uint32_t DefaultPumpRampUpMillis = 4000;

const uint32_t CycleLengthMillis = 40;

const uint32_t Max31856ReadIntervalCycles = 2;
const uint32_t Xdb401ReadIntervalCycles = 1;
#define HEATING_CYCLE_LENGTH 10
#define PID_INTERVAL_CYCLES 25

#define STEAM_WATER_SUPPLY_MIN_TEMPERATURE 105
#define STEAM_WATER_SUPPLY_INTERVAL_CYCLES 32

#define FLOW_PROCESS_INTERVAL_CYCLES 1


#define SPLASH_IMAGE_DURATION 10000





#define HEATING_ENERGY_PER_ML_AND_KELVIN_WATTSECONDS 5.76



#define READY_NOTIFICATION_INTERVAL 60000

#define BLUE 0x1e64ae
#define TURQUOISE 0x2c9699
#define GREEN 0x8eb333
#define YELLOW 0xf3ce2f
#define RED 0xd42223

unsigned int const heatGradient[] = { 0x7f7f7f, BLUE, TURQUOISE, GREEN, YELLOW, RED };
static float pressureHeatWeights[] = { 1.0f, 5.0f, 2.0f, 2.0f, 1.0f };
static float temperatureHeatWeights[] = { 5.0f, 0.0f, 2.0f, 2.0f, 3.0f };
static float brewingUnitTemperatureHeatWeights[] = { 5.0f, 0.0f, 5.0f, 60.0f, 15.0f };

unsigned int const waterLevelGradientColors[] = {RED, YELLOW,GREEN };
static float waterLevelHeatWeights[] = { 0.2f, 0.8f };