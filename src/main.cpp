#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_VL53L0X.h>
#include <DataTome.h>
#include <FS.h>
#include <LittleFS.h>
#include <PID_v1.h>

#include <vector>

#include "Constants.h"
#include "ColorScale.h"
#include "Fonts.h"
#include "Gc9a01Display.h"
#include "SolidStateRelay.h"
#include "LeadingEdgeDimmer.h"
#include "Xdb401PressureSensor.h"
#include "Mlx90614TemperatureSensor.h"
#include "PulseCounter.h"
#include "MedianAverage.h"
#include "JitterFilter.h"
#include "BleServer.h"


Gc9a01Display display(Gc9a01SpiWriteFreq, PinGc9a01Sclk, PinGc9a01Mosi, PinGc9a01Dc, PinGc9a01Cs, PinGc9a01Rst);

AcZeroCrossDetector zeroCrossDetector(PinAcZerocross);
SolidStateRelay heatingRelay(PinHeatingAc, zeroCrossDetector);
LeadingEdgeDimmer pumpDimmer(PinPumpAc, zeroCrossDetector);
Xdb401PressureSensor pressureSensor(Wire, 20.0);
Mlx90614TemperatureSensor brewingUnitTemperatureSensor(Wire);
PulseCounter flowCounter(PinFlowMeter, 20);

BleServer bleServer;

SPIClass hspi(HSPI);
Adafruit_MAX31865 thermo(PinMax31865Cs, &hspi);
Adafruit_VL53L0X waterLevelSensor = Adafruit_VL53L0X();

DataTomeMvAvg<float, double> temperateAvg(20), brewingUnitTemperateAvg(20), pressureAvg(25), flowWeightAvg(4), flowVolumeAvg(4), waterLevelAvg(10);

MedianAverage<uint8_t, 9> waterLevelAverage;

double temperatureSet, temperatureIs, pidOut, brewingUnitTemperature;

PID temperaturePid(&temperatureIs, &pidOut, &temperatureSet, DefaultPidP, 0, 0, DIRECT);

ColorScale tempGradient(heatGradient, temperatureHeatWeights, 6);
ColorScale brewingUnitTempGradient(heatGradient, brewingUnitTemperatureHeatWeights, 6);
ColorScale pressureGradient(heatGradient, pressureHeatWeights, 6);
ColorScale waterLevelGradient(waterLevelGradientColors, waterLevelHeatWeights, 2);

JitterFilter<int, 3> temperatureFlappingFilter(5000);
JitterFilter<int, 3> brewingUnitTemperatureFlappingFilter(5000);
JitterFilter<int, 3> waterLevelFlappingFilter(300000);

bool waterLevelSensorPresent;

std::vector<fs::File> splashFiles;

bool scaleConnected;
int32_t scaleValue;

int32_t lastScaleValue = 0;
uint32_t lastScaleValueTimestamp = 0;

bool infusionStopped;

void getSplashImages()
{
  fs::File root = LittleFS.open("/"); 

  while (fs::File file = root.openNextFile()) 
  {
      if (!strncmp(file.name(), "splash-", 7))
      {
        splashFiles.push_back(file);
      }
  }
}

unsigned char splashBuf[256];

void setTemperature(float t)
{
  temperatureSet = t;
  temperatureHeatWeights[1] = temperatureSet - temperatureHeatWeights[0] - temperatureHeatWeights[2];
}

void setBrewingUnitTemperature(float t)
{
  brewingUnitTemperature = t;
  brewingUnitTemperatureHeatWeights[1] = brewingUnitTemperature - brewingUnitTemperatureHeatWeights[0] - brewingUnitTemperatureHeatWeights[2];
}

unsigned int flowCounterValue = 0;
unsigned int lastFlowCounterValue = 0;
uint32_t lastFlowCounterTimestamp = 0;

lv_obj_t *standbyScreen;
lv_obj_t *standbyTemperatureArc;
lv_obj_t *standbyTemperatureLabel;

lv_obj_t *brewingUnitTemperatureArc;
lv_obj_t *brewingUnitTemperatureLabel;

lv_obj_t *waterLevelArc;
lv_obj_t *waterLevelLabel;
lv_obj_t *waterLevelSymbol;

lv_obj_t *scaleValueLabel;
lv_obj_t *scaleUnitLabel;

lv_obj_t *infuseScreen;
lv_obj_t *infusePressureArc;
lv_obj_t *infusePressureLabel;
lv_obj_t *infuseTemperatureLabel;
lv_obj_t *infuseFlowLabel;
lv_obj_t *infuseFlowUnitLabel;
lv_obj_t *infuseVolumeLabel;
lv_obj_t *infuseWeightLabel;
lv_obj_t *infuseWeightUnitLabel;


lv_obj_t *pairingWaitScreen;
lv_obj_t *pairingPinScreen;
lv_obj_t *pairingFailureScreen;
lv_obj_t *pairingSuccessScreen;

lv_obj_t *pairingPinLabel;

lv_obj_t *confirmHintLabel;


void initStandbyUi()
{
  standbyScreen = lv_obj_create(NULL);
  standbyTemperatureArc = lv_arc_create(standbyScreen);
  lv_obj_set_size(standbyTemperatureArc, 230, 230);
  lv_obj_set_style_arc_width(standbyTemperatureArc, 12, LV_PART_MAIN);
  lv_obj_set_style_arc_width(standbyTemperatureArc, 12, LV_PART_INDICATOR);
  lv_arc_set_rotation(standbyTemperatureArc, 145);
  lv_arc_set_bg_angles(standbyTemperatureArc, 0, 250);
  lv_obj_remove_style(standbyTemperatureArc, NULL, LV_PART_KNOB);
  lv_obj_center(standbyTemperatureArc);

  standbyTemperatureLabel = lv_label_create(standbyScreen);
  lv_obj_set_style_text_font(standbyTemperatureLabel, &lv_font_my_40, 0);
  lv_obj_set_width(standbyTemperatureLabel, 150);
  lv_obj_set_style_text_align(standbyTemperatureLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(standbyTemperatureLabel, LV_ALIGN_CENTER, 0, -56);

  brewingUnitTemperatureArc = lv_arc_create(standbyScreen);
  lv_obj_set_size(brewingUnitTemperatureArc, 200, 200);
  lv_obj_set_style_arc_width(brewingUnitTemperatureArc, 8, LV_PART_MAIN);
  lv_obj_set_style_arc_width(brewingUnitTemperatureArc, 8, LV_PART_INDICATOR);
  lv_arc_set_rotation(brewingUnitTemperatureArc, 145);
  lv_arc_set_bg_angles(brewingUnitTemperatureArc, 0, 150);
  lv_obj_remove_style(brewingUnitTemperatureArc, NULL, LV_PART_KNOB);
  lv_obj_center(brewingUnitTemperatureArc);

  waterLevelArc = lv_arc_create(standbyScreen);
  lv_obj_set_size(waterLevelArc, 200, 200);
  lv_obj_set_style_arc_width(waterLevelArc, 8, LV_PART_MAIN);
  lv_obj_set_style_arc_width(waterLevelArc, 8, LV_PART_INDICATOR);
  lv_arc_set_rotation(waterLevelArc, 305);
  lv_arc_set_bg_angles(waterLevelArc, 0, 90);
  lv_obj_remove_style(waterLevelArc, NULL, LV_PART_KNOB);
  lv_obj_center(waterLevelArc);

  brewingUnitTemperatureLabel = lv_label_create(standbyScreen);
  lv_obj_set_style_text_font(brewingUnitTemperatureLabel, &lv_font_my_32, 0);
  lv_obj_set_width(brewingUnitTemperatureLabel, 160);
  lv_obj_set_style_text_align(brewingUnitTemperatureLabel, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_align(brewingUnitTemperatureLabel, LV_ALIGN_CENTER, 0, -23);

  waterLevelSymbol = lv_label_create(standbyScreen);
  lv_obj_set_style_text_font(waterLevelSymbol, &lv_font_my_20, 0);
  lv_obj_set_width(waterLevelSymbol, 160);
  lv_obj_set_style_text_align(waterLevelSymbol, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_align(waterLevelSymbol, LV_ALIGN_CENTER, 0, -23);
  lv_label_set_text_fmt(waterLevelSymbol, "\xEF\x81\x83");

  waterLevelLabel = lv_label_create(standbyScreen);
  lv_obj_set_style_text_font(waterLevelLabel, &lv_font_my_32, 0);
  lv_obj_set_width(waterLevelLabel, 126);
  lv_obj_set_style_text_align(waterLevelLabel, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_align(waterLevelLabel, LV_ALIGN_CENTER, 0, -23);

  scaleValueLabel = lv_label_create(standbyScreen);
  lv_obj_set_style_text_font(scaleValueLabel, &lv_font_my_68, 0);
  lv_obj_set_width(scaleValueLabel, 230);
  lv_obj_set_style_text_align(scaleValueLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(scaleValueLabel, LV_ALIGN_CENTER, 0, 32);

  scaleUnitLabel = lv_label_create(standbyScreen);
  lv_obj_set_style_text_font(scaleUnitLabel, &lv_font_my_20, 0);
  lv_obj_set_width(scaleUnitLabel, 230);
  lv_obj_set_style_text_align(scaleUnitLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(scaleUnitLabel, LV_ALIGN_CENTER, 0, 70);
  lv_label_set_text_fmt(scaleUnitLabel, "Gramm");

}

void initInfuseUi()
{
  infuseScreen = lv_obj_create(NULL);

  infusePressureArc = lv_arc_create(infuseScreen);
  lv_obj_set_size(infusePressureArc, 230, 230);
  lv_obj_set_style_arc_width(infusePressureArc, 16, LV_PART_MAIN);
  lv_obj_set_style_arc_width(infusePressureArc, 16, LV_PART_INDICATOR);
  lv_arc_set_rotation(infusePressureArc, 145);
  lv_arc_set_bg_angles(infusePressureArc, 0, 250);
  lv_obj_remove_style(infusePressureArc, NULL, LV_PART_KNOB);
  lv_obj_center(infusePressureArc);

  infusePressureLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(infusePressureLabel, &lv_font_my_48, 0);
  lv_obj_set_width(infusePressureLabel, 144);
  lv_obj_set_style_text_align(infusePressureLabel, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_align(infusePressureLabel, LV_ALIGN_TOP_LEFT, 0, 26);


  lv_obj_t *barLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(barLabel, &lv_font_my_20, 0);
  lv_obj_set_width(barLabel, 50);
  lv_obj_set_style_text_align(barLabel, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_align(barLabel, LV_ALIGN_TOP_LEFT, 146, 49);
  lv_label_set_text_fmt(barLabel, "bar");


  infuseTemperatureLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(infuseTemperatureLabel, &lv_font_my_32, 0);
  lv_obj_set_width(infuseTemperatureLabel, 114);
  lv_obj_set_style_text_align(infuseTemperatureLabel, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_align(infuseTemperatureLabel, LV_ALIGN_TOP_LEFT, 0, 70);


  infuseVolumeLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(infuseVolumeLabel, &lv_font_my_32, 0);
  lv_obj_set_width(infuseVolumeLabel, 70);
  lv_obj_set_style_text_align(infuseVolumeLabel, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_align(infuseVolumeLabel, LV_ALIGN_TOP_LEFT, 110, 70);

  lv_obj_t *mlLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(mlLabel, &lv_font_my_20, 0);
  lv_obj_set_width(mlLabel, 50);
  lv_obj_set_style_text_align(mlLabel, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_align(mlLabel, LV_ALIGN_TOP_LEFT, 182, 80);
  lv_label_set_text_fmt(mlLabel, "ml");


  infuseWeightLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(infuseWeightLabel, &lv_font_my_68, 0);
  lv_obj_set_width(infuseWeightLabel, 230);
  lv_obj_set_style_text_align(infuseWeightLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(infuseWeightLabel, LV_ALIGN_CENTER, 0, 54);

  infuseWeightUnitLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(infuseWeightUnitLabel, &lv_font_my_20, 0);
  lv_obj_set_width(infuseWeightUnitLabel, 230);
  lv_obj_set_style_text_align(infuseWeightUnitLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(infuseWeightUnitLabel, LV_ALIGN_CENTER, 0, 92);

  infuseFlowLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(infuseFlowLabel, &lv_font_my_48, 0);
  lv_obj_set_width(infuseFlowLabel, 144);
  lv_obj_set_style_text_align(infuseFlowLabel, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_align(infuseFlowLabel, LV_ALIGN_TOP_LEFT, 0, 97);


  infuseFlowUnitLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(infuseFlowUnitLabel, &lv_font_my_20, 0);
  lv_obj_set_width(infuseFlowUnitLabel, 50);
  lv_obj_set_style_text_align(infuseFlowUnitLabel, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_align(infuseFlowUnitLabel, LV_ALIGN_TOP_LEFT, 146, 120);

}

void initPairingUi(char *btDeviceName)
{
  pairingWaitScreen = lv_obj_create(NULL);

  lv_obj_t *symbolLabel = lv_label_create(pairingWaitScreen);
  lv_obj_set_style_text_font(symbolLabel, &lv_font_my_48, 0);
  lv_obj_set_width(symbolLabel, 230);
  lv_obj_set_style_text_align(symbolLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(symbolLabel, LV_ALIGN_CENTER, 0, -80);
  lv_label_set_text_fmt(symbolLabel, LV_SYMBOL_BLUETOOTH);

  lv_obj_t *pairingLabel = lv_label_create(pairingWaitScreen);
  lv_obj_set_style_text_font(pairingLabel, &lv_font_my_32, 0);
  lv_obj_set_width(pairingLabel, 230);
  lv_obj_set_style_text_align(pairingLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(pairingLabel, LV_ALIGN_CENTER, 0, -18);
  lv_label_set_text_fmt(pairingLabel, "Kopplung\naktiv");

  lv_obj_t *deviceNameLabel = lv_label_create(pairingWaitScreen);
  lv_obj_set_style_text_font(deviceNameLabel, &lv_font_my_20, 0);
  lv_obj_set_width(deviceNameLabel, 190);
  lv_obj_set_style_text_align(deviceNameLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(deviceNameLabel, LV_ALIGN_CENTER, 0, 50);
  lv_label_set_text_fmt(deviceNameLabel, "Name: %s", btDeviceName);

  pairingPinScreen = lv_obj_create(NULL);

  lv_obj_t *pinLabel = lv_label_create(pairingPinScreen);
  lv_obj_set_style_text_font(pinLabel, &lv_font_my_36, 0);
  lv_obj_set_width(pinLabel, 230);
  lv_obj_set_style_text_align(pinLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(pinLabel, LV_ALIGN_CENTER, 0, -85);
  lv_label_set_text_fmt(pinLabel, "PIN:");

  pairingPinLabel = lv_label_create(pairingPinScreen);
  lv_obj_set_style_text_font(pairingPinLabel, &lv_font_my_48, 0);
  lv_obj_set_width(pairingPinLabel, 230);
  lv_obj_set_style_text_align(pairingPinLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(pairingPinLabel, LV_ALIGN_CENTER, 0, -45);

  confirmHintLabel = lv_label_create(pairingPinScreen);
  lv_obj_set_style_text_font(confirmHintLabel, &lv_font_my_20, 0);
  lv_obj_set_width(confirmHintLabel, 230);
  lv_obj_set_style_text_align(confirmHintLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(confirmHintLabel, LV_ALIGN_CENTER, 0, 40);
  lv_label_set_text_fmt(confirmHintLabel, "Übereinstimmung der PINs durch Um- legen des Brüh- oder Dampfschalters bestätigen!");

  pairingSuccessScreen = lv_obj_create(NULL);

  symbolLabel = lv_label_create(pairingSuccessScreen);
  lv_obj_set_style_text_font(symbolLabel, &lv_font_my_48, 0);
  lv_obj_set_width(symbolLabel, 230);
  lv_obj_set_style_text_align(symbolLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(symbolLabel, LV_ALIGN_CENTER, 0, -65);
  lv_label_set_text_fmt(symbolLabel, LV_SYMBOL_OK);

  lv_obj_t *successLabel = lv_label_create(pairingSuccessScreen);
  lv_obj_set_style_text_font(successLabel, &lv_font_my_36, 0);
  lv_obj_set_width(successLabel, 230);
  lv_obj_set_style_text_align(successLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(successLabel, LV_ALIGN_CENTER, 0, -5);
  lv_label_set_text_fmt(successLabel, "Kopplung\nerfolgreich");

  lv_obj_t *successText = lv_label_create(pairingSuccessScreen);
  lv_obj_set_style_text_font(successText, &lv_font_my_20, 0);
  lv_obj_set_width(successText, 230);
  lv_obj_set_style_text_align(successText, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(successText, LV_ALIGN_CENTER, 0, 65);
  lv_label_set_text_fmt(successText, "Maschine neu\nstarten!");


  pairingFailureScreen = lv_obj_create(NULL);

  symbolLabel = lv_label_create(pairingFailureScreen);
  lv_obj_set_style_text_font(symbolLabel, &lv_font_my_48, 0);
  lv_obj_set_width(symbolLabel, 230);
  lv_obj_set_style_text_align(symbolLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(symbolLabel, LV_ALIGN_CENTER, 0, -70);
  lv_label_set_text_fmt(symbolLabel, LV_SYMBOL_WARNING);

  lv_obj_t *failureText = lv_label_create(pairingFailureScreen);
  lv_obj_set_style_text_font(failureText, &lv_font_my_20, 0);
  lv_obj_set_width(failureText, 210);
  lv_obj_set_style_text_align(failureText, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(failureText, LV_ALIGN_CENTER, 0, 20);
  lv_label_set_text_fmt(failureText, "Kopplung wurde abgelehnt oder nicht rechtzeitig bestätigt. Maschine neu starten!");
}

bool infusing = false;
bool steam = false;
bool hotWater = false;
unsigned int startupTime = 0;

uint32_t lastSplashDisplayTime = 0;
uint32_t currentSplash = 0;
uint32_t currentSplashPos = 0;

lv_area_t excluded = {
    56,
    112,
    184,
    240
};

void updateUi();

void initLvgl()
{
  display.init();
  display.setRotation(1);
  lv_init();
  lv_disp_drv_register(&display.lvglDriver());
}

void lvglUpdateTaskFunc(void *parameter)
{
  initLvgl();
  initStandbyUi();
  initInfuseUi();
  lv_scr_load(standbyScreen);

  getSplashImages();

  for (;;)
  {
    vTaskSuspend(NULL);
    unsigned long start = millis();

    bool displaySplash = !infusing && !steam && !hotWater && !splashFiles.empty() && !scaleConnected;
    if (!displaySplash)
    {
      display.clearLvglExcludedArea();
    }

    updateUi();
    lv_timer_handler();

    if (displaySplash)
    {
      display.setLvglExlucdedArea(excluded);
      unsigned long now = millis();
      if (lastSplashDisplayTime == 0 || now > lastSplashDisplayTime + SPLASH_IMAGE_DURATION)
      {
        currentSplash = (currentSplash + 1) % splashFiles.size();
        currentSplashPos = 0;
        lastSplashDisplayTime = now;
      }
      if (currentSplashPos < 16)
      {
        for (int i = 0; i < 8; ++i)
        {
          int row = i * 16 + currentSplashPos;
          splashFiles[currentSplash].seek(row * 256);
          splashFiles[currentSplash].read(splashBuf, 256);
          display.pushImageDMA(56, 110 + row, 128, 1, (uint16_t *)splashBuf);
        }
        currentSplashPos++;
      }
    }
  }
}

TaskHandle_t lvglUpdateTask;

#define CONFIG_VERSION    0x0001

struct Qm3032Config
{
  uint16_t version;
  float temperature;
  float waterTemperature;
  float pumpPower;
  float UNUSED_preinfusionVolume;
  uint16_t preinfusionDuration;
  float preinfusionPressure;
  float steamTemperature;
  uint8_t steamWaterSupplyCycles;
  float brewingUnitTemperature;
  char btDeviceName[32];
  float volumeBasedHeatingFactor;
  uint16_t waterLevelMax;
  uint16_t waterLevelMin;
  float preinfusionPumpPower;
  float hotWaterPumpPower;
  float maxInfusionVolume;
  float infusionWeight;
};

struct Qm3032Config defaultConfig = { 1, 94.0, 20.0, 0.70, 8.0, 12000, 1.8, 125.0, 3, 70.0, { 0 }, 0.7, 20, 240, 0.48, 0.35, 40.0, 27.0 };

bool readConfig(struct Qm3032Config &config)
{
  fs::File file = LittleFS.open("/config.bin", "r"); 
  if (file)
  {
    bool success = file.read(reinterpret_cast<uint8_t *>(&config), sizeof(struct Qm3032Config)) <= sizeof(struct Qm3032Config);
    file.close();
    return success;
  }
  return false;
}

bool writeConfig(const struct Qm3032Config &config)
{
  fs::File file = LittleFS.open("/config.bin", "w"); 
  if (file)
  {    
    file.write(reinterpret_cast<const uint8_t *>(&config), sizeof(struct Qm3032Config));
    file.close();
  }
  return false;
}

Qm3032Config config;

void readyMelody()
{
  tone(PinBuzzer, 1056, 150);
  tone(PinBuzzer, 0, 20);
  tone(PinBuzzer, 792, 150);
}

bool probeDevice(TwoWire &wire, uint8_t address) {
  wire.beginTransmission(address);
  return wire.endTransmission() == 0;
}

void setup()
{
  Serial.begin(115200);

  analogWriteFrequency(PinGc9a01Backlight, 2000);
  analogWrite(PinGc9a01Backlight, 0);

  LittleFS.begin();

  pinMode(PinInfuseSwitch, INPUT);
  pinMode(PinSteamSwitch, INPUT);
  pinMode(PinHotwaterSwitch, INPUT);

  infusing = digitalRead(PinInfuseSwitch);
  steam = digitalRead(PinSteamSwitch);
  hotWater = digitalRead(PinHotwaterSwitch);

  if (infusing || steam)
  {
    // TODO pairing/bonding
  }
  else
  {
    xTaskCreatePinnedToCore(lvglUpdateTaskFunc, "lvglUpdateTask", 10000, NULL, 1, &lvglUpdateTask, 0);
  }

  config = defaultConfig;
  //if (!readConfig(config))
  //{
  //  Serial.printf("Read config failed\n");
  //}

  Wire.begin();

  heatingRelay.begin();
  pumpDimmer.begin();
  zeroCrossDetector.begin();

  waterLevelSensorPresent = probeDevice(Wire, VL53L0X_I2C_ADDR);

  if (waterLevelSensorPresent) 
  {
    waterLevelSensor.begin();
    waterLevelSensor.configSensor(Adafruit_VL53L0X::VL53L0X_Sense_config_t::VL53L0X_SENSE_HIGH_ACCURACY);
    waterLevelSensor.startRange();
  }

  hspi.begin(PinMax31865Sclk, PinMax31865Miso, PinMax31865Mosi);
  
  thermo.begin(MAX31865_3WIRE);
  thermo.enable50Hz(true);
  thermo.autoConvert(true);
  thermo.enableBias(true);

  pinMode(PinValveAc, OUTPUT);

  temperaturePid.SetOutputLimits(0, PidMaxOutput);
  temperaturePid.SetSampleTime(PID_INTERVAL_CYCLES * CycleLengthMillis);
  temperaturePid.SetMode(AUTOMATIC);

  flowCounter.begin();

  setTemperature(config.temperature);
  setBrewingUnitTemperature(config.brewingUnitTemperature);

  bleServer.start("SchorschCoff");

  startupTime = millis();
}

unsigned long cycle = 0;
unsigned long valveDeadline;

unsigned long infuseStart;

unsigned int splashCurrent = 0;

bool warmup = true;

unsigned int readyCycleCount = 0;

unsigned int flowCounterInfusionStart;

unsigned int infusionHeatingCyclesIs;

int32_t scaleValueInfusionStart;

bool preinfusionPressureReached;
bool preinfusionPassed;

bool coldFlush;

uint8_t waterLevel = 0;

void updateUi()
{
  float temperatureAvgDegree = temperateAvg.get();

  if (infusing || steam || hotWater)
  {
    int tempOffsetDialStart;
    int tempOffsetDialAmount;

    float tempDiff = min(1.0, max(-1.0, (temperatureAvgDegree - temperatureSet) / 5.0));

    float displayedPressure = max(0.0f, pressureAvg.get());

    if (lv_scr_act() != infuseScreen)
    {
      lv_scr_load(infuseScreen);
    }

    int temperatureAvgDegreeTenths = (int)(temperatureAvgDegree * 10);
    if (temperatureAvgDegreeTenths < 1000)
    {
      lv_label_set_text_fmt(infuseTemperatureLabel, "%d.%d°", temperatureAvgDegreeTenths / 10, temperatureAvgDegreeTenths % 10);
    }
    else
    {
      lv_label_set_text_fmt(infuseTemperatureLabel, "%d°", temperatureAvgDegreeTenths / 10);
    }

    int32_t pressureTenthsBar = (int32_t)(displayedPressure * 10);
    if (pressureTenthsBar < 100)
    {
      lv_label_set_text_fmt(infusePressureLabel, "%d.%d", pressureTenthsBar / 10, pressureTenthsBar % 10);
    }
    else
    {
      lv_label_set_text_fmt(infusePressureLabel, "%d", pressureTenthsBar / 10);
    }
    
    lv_arc_set_angles(infusePressureArc, 0, displayedPressure / 16.0 * 250);
    lv_obj_set_style_arc_color(infusePressureArc, lv_color_hex(pressureGradient.getRgb(displayedPressure)), LV_PART_INDICATOR | LV_STATE_DEFAULT );

    int32_t volumeTenthMl = (int32_t)((flowCounter.ticks() - flowCounterInfusionStart) * FlowMeterVolumePerTickMilliliters * 10.0);

    if (scaleConnected)
    {
      int32_t weight = scaleValue - scaleValueInfusionStart;
      if (weight >= 0)
      {
        lv_label_set_text_fmt(infuseWeightUnitLabel, "Gramm");
        if (weight < 1000)
        {
          lv_label_set_text_fmt(infuseWeightLabel, "%d.%d", weight / 10, weight % 10);
        }
        else
        {
          lv_label_set_text_fmt(infuseWeightLabel, "%d", weight / 10);
        }
        lv_label_set_text_fmt(infuseFlowUnitLabel, "g/s");
        lv_label_set_text_fmt(infuseFlowLabel, "%.1f", flowWeightAvg.get());
      }
    }
    else
    {
      lv_label_set_text_fmt(infuseWeightUnitLabel, "ml");
      if (volumeTenthMl < 1000)
      {
        lv_label_set_text_fmt(infuseWeightLabel, "%d.%d", volumeTenthMl / 10, volumeTenthMl % 10);
      }
      else
      {
        lv_label_set_text_fmt(infuseWeightLabel, "%d", volumeTenthMl / 10);
      }
      lv_label_set_text_fmt(infuseFlowUnitLabel, "ml/s");
      lv_label_set_text_fmt(infuseFlowLabel, "%.1f", flowVolumeAvg.get());
    }

    if (volumeTenthMl < 1000)
    {
      lv_label_set_text_fmt(infuseVolumeLabel, "%d.%d", volumeTenthMl / 10, volumeTenthMl % 10);
    }
    else
    {
      lv_label_set_text_fmt(infuseVolumeLabel, "%d", volumeTenthMl / 10);
    }
  }
  else
  {
    float brewingUnitTemperatureAvgDegree = brewingUnitTemperateAvg.get();

    if (lv_scr_act() != standbyScreen)
    {
      lv_scr_load(standbyScreen);
    }

    int filteredTemperature = temperatureFlappingFilter.apply((int)(temperatureAvgDegree * 10));
    if (filteredTemperature < 1000)
    {
      lv_label_set_text_fmt(standbyTemperatureLabel, "%d.%d°", filteredTemperature / 10, filteredTemperature % 10);
    }
    else
    {
      lv_label_set_text_fmt(standbyTemperatureLabel, "%d°", filteredTemperature / 10);
    }

    lv_arc_set_angles(standbyTemperatureArc, 0, temperatureAvgDegree / TemperatureSafetyGuardCelsius * 250);
    lv_obj_set_style_arc_color(standbyTemperatureArc, lv_color_hex(tempGradient.getRgb(temperatureAvgDegree)), LV_PART_INDICATOR | LV_STATE_DEFAULT );

    int filteredBrewingUnitTemperature = brewingUnitTemperatureFlappingFilter.apply((int)brewingUnitTemperatureAvgDegree);
    lv_label_set_text_fmt(brewingUnitTemperatureLabel, "%d°", filteredBrewingUnitTemperature);

    lv_arc_set_angles(brewingUnitTemperatureArc, 0, brewingUnitTemperatureAvgDegree / 80 * 150);
    lv_obj_set_style_arc_color(brewingUnitTemperatureArc, lv_color_hex(brewingUnitTempGradient.getRgb(brewingUnitTemperatureAvgDegree)), LV_PART_INDICATOR | LV_STATE_DEFAULT );

    if (waterLevel > 0)
    {
      float waterLevelRatio = (float)(config.waterLevelMin - waterLevel) / (config.waterLevelMin - config.waterLevelMax);
      waterLevelRatio = max(0.0f, min(1.0f, waterLevelRatio));
      bool warn = waterLevelRatio < 0.15;
      bool blink = warn && cycle % 16 < 8;

      lv_obj_clear_flag(waterLevelLabel, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(waterLevelArc, LV_OBJ_FLAG_HIDDEN);
      if (blink)
      {
        lv_obj_add_flag(waterLevelSymbol, LV_OBJ_FLAG_HIDDEN);
      }
      else
      {
        lv_obj_clear_flag(waterLevelSymbol, LV_OBJ_FLAG_HIDDEN);
      }

      lv_color_t color = lv_color_hex(waterLevelGradient.getRgb(waterLevelRatio));

      lv_arc_set_angles(waterLevelArc, 90 - (waterLevelRatio * 90), 90);
      lv_obj_set_style_arc_color(waterLevelArc, color, LV_PART_INDICATOR | LV_STATE_DEFAULT );
      
      int filteredWaterLevelPercent = waterLevelFlappingFilter.apply((int)(waterLevelRatio*100));
      lv_label_set_text_fmt(waterLevelLabel, "%d%%", filteredWaterLevelPercent);
    }
    else
    {
      lv_obj_add_flag(waterLevelSymbol, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(waterLevelArc, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(waterLevelLabel, LV_OBJ_FLAG_HIDDEN);
    }

    if (scaleConnected)
    {
      lv_obj_clear_flag(scaleUnitLabel, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(scaleValueLabel, LV_OBJ_FLAG_HIDDEN);
      if (abs(scaleValue) < 1000)
      {
        lv_label_set_text_fmt(scaleValueLabel, "%d.%d", scaleValue / 10, abs(scaleValue % 10));
      }
      else
      {
        lv_label_set_text_fmt(scaleValueLabel, "%d", scaleValue / 10);
      }
    }
    else
    {
      lv_obj_add_flag(scaleUnitLabel, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(scaleValueLabel, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

 void loop()
{
  if (cycle < 64)
  {
    analogWrite(PinGc9a01Backlight, cycle * 4);
  }
  else if (cycle == 64)
  {
    pinMode(PinGc9a01Backlight, OUTPUT);
    digitalWrite(PinGc9a01Backlight, 1);
  }

  if (steam)
  {
    if (cycle % Max31856ReadIntervalCycles == 0)
    {
      if (temperatureIs < config.steamTemperature)
      {
        heatingRelay.setCycles(Max31856ReadIntervalCycles * CycleLengthMillis / HEATING_CYCLE_LENGTH);
      }
      else
      {
        heatingRelay.setCycles(0, true);
      }
    }
  }
  else if (!infusing && !hotWater)
  {
    if (temperatureIs < MaxTemperatureCelsius)
    {        
      if (temperaturePid.Compute())
      {
        heatingRelay.setCycles(pidOut / PidMaxOutput * (PID_INTERVAL_CYCLES * CycleLengthMillis / HEATING_CYCLE_LENGTH));
      }
    }
    else 
    {
      heatingRelay.setCycles(0, true);
    }
  }

  if (eTaskGetState(lvglUpdateTask) == eTaskState::eSuspended)
  {
    vTaskResume(lvglUpdateTask);
  }

  if (!steam && !hotWater && digitalRead(PinInfuseSwitch) != infusing)
  {
    infusing = !infusing;
    if (infusing)
    {
      digitalWrite(PinValveAc, HIGH);
      valveDeadline = 0;
      infuseStart = millis();
      flowCounterInfusionStart = flowCounter.ticks();
      scaleValueInfusionStart = scaleValue;
      infusionHeatingCyclesIs = 0;
      preinfusionPressureReached = false;
      preinfusionPassed = false;
      infusionStopped = false;
      flowWeightAvg.clear();
      flowVolumeAvg.clear();
    }
    else
    {
      heatingRelay.setCycles(0);

      setTemperature(config.temperature);

      pumpDimmer.setPowerLevel(0);
      valveDeadline = millis() + (millis() - infuseStart < 10000 ? 20000 : 2000);

      currentSplashPos = 0;
    }
  }

  if (!infusing && !hotWater && digitalRead(PinSteamSwitch) != steam)
  {
    steam = !steam;
    if (steam)
    {
      digitalWrite(PinValveAc, HIGH);
      valveDeadline = 0;
    }
    else
    {
      pumpDimmer.setPowerLevel(0);
      digitalWrite(PinValveAc, LOW);
    }
  }

  if (!infusing && !steam && digitalRead(PinHotwaterSwitch) != hotWater)
  {
    hotWater = !hotWater;
    if (hotWater)
    {
      coldFlush = temperateAvg.get() >= 100.0;
      digitalWrite(PinValveAc, coldFlush ? LOW : HIGH);
      flowCounterInfusionStart = flowCounter.ticks();
      infusionHeatingCyclesIs = 0;
    }
    else
    {
      setTemperature(config.temperature);
      pumpDimmer.setPowerLevel(0);
      digitalWrite(PinValveAc, LOW);
    }
  }

  if (infusing)
  {
      float pumpValue;
      unsigned int infusionTime = millis() - infuseStart;
      if (!preinfusionPassed)
      {
        if (infusionTime < config.preinfusionDuration)
        {
          pumpValue = preinfusionPressureReached ? 0.0 : config.preinfusionPumpPower;
        }
        else
        {
          preinfusionPassed = true;
          flowCounterInfusionStart = flowCounter.ticks();
          infusionHeatingCyclesIs = 0;
          infuseStart = millis();
        }
      }
      else
      {
        if (!infusionStopped)
        {
          infusionStopped = scaleConnected ? 
            scaleValue - scaleValueInfusionStart >= config.infusionWeight * 10 : 
            (flowCounter.ticks() - flowCounterInfusionStart) * FlowMeterVolumePerTickMilliliters >= config.maxInfusionVolume;
        }

        if (infusionStopped)
        {
          pumpValue = 0;
          digitalWrite(PinValveAc, LOW);
        }
        else if (infusionTime < DefaultPumpRampUpMillis)
        {
          pumpValue = config.preinfusionPumpPower + (float)(infusionTime) / DefaultPumpRampUpMillis * (config.pumpPower - config.preinfusionPumpPower);
        }
        else
        {
          pumpValue = config.pumpPower;
        }
      }

      pumpDimmer.setPowerLevel(pumpValue);
  }
  else if (steam)
  {
      if (cycle % STEAM_WATER_SUPPLY_INTERVAL_CYCLES == 0 && temperatureIs > STEAM_WATER_SUPPLY_MIN_TEMPERATURE)
      {
         pumpDimmer.setPowerLevel(config.preinfusionPumpPower);
      }
      else if (cycle % STEAM_WATER_SUPPLY_INTERVAL_CYCLES == config.steamWaterSupplyCycles)
      {
         pumpDimmer.setPowerLevel(0);
      }
  }
  else if (hotWater)
  {
      pumpDimmer.setPowerLevel(config.hotWaterPumpPower);
  }
  else
  {
    if (valveDeadline != 0 && millis() > valveDeadline) 
    {
      digitalWrite(PinValveAc, LOW);
      valveDeadline = 0;
    }
  }

  if (cycle % Max31856ReadIntervalCycles == 0)
  {
    temperatureIs = thermo.calculateTemperature(thermo.readRTDCont(), Max31865ReferenceTemperature, Max31865ReferenceResistorValueOhms);

    temperateAvg.push(temperatureIs);

    if (warmup && config.temperature - temperateAvg.get() < WarmupTemperatureThresholdKelvin)
    {
      temperaturePid.SetTunings(DefaultPidP, DefaultPidI, DefaultPidD);
      warmup = false;
    }
  
    double brewingUnitTemperature;
    if (brewingUnitTemperatureSensor.readValue(Mlx90614TemperatureSensor::Register::Obj1, brewingUnitTemperature) == 0)
    {
      brewingUnitTemperateAvg.push(brewingUnitTemperature);
    }
    else
    {
      // TODO
    }
  }

  if (cycle % Xdb401ReadIntervalCycles == 0)
  {
    double pressure;
    if (pressureSensor.readValue(pressure) == 0)
    {
        pressureAvg.push(pressure);

        if (infusing && !preinfusionPressureReached && pressureAvg.get() > config.preinfusionPressure)
        {
          preinfusionPressureReached = true;
        }
    }
  }

  uint32_t currentFlowCounter;
  uint32_t currentFlowCounterTimestamp;
  flowCounter.ticks(currentFlowCounter, currentFlowCounterTimestamp);
  if (currentFlowCounter > flowCounterValue)
  {
    if (millis() - lastScaleValueTimestamp >= 500)
    {
      float flow = (currentFlowCounter - lastFlowCounterValue) * FlowMeterVolumePerTickMilliliters / ((currentFlowCounterTimestamp - lastFlowCounterTimestamp) / 1000.0);
      flowVolumeAvg.push(flow);
      lastFlowCounterValue = currentFlowCounter;
      lastFlowCounterTimestamp = currentFlowCounterTimestamp;
    }
    flowCounterValue = currentFlowCounter;

    if (infusing || hotWater)
    {
        if (temperateAvg.get() < MaxTemperatureCelsius && !(hotWater && coldFlush)) 
        {
          float infusionVolume = (currentFlowCounter - flowCounterInfusionStart) * FlowMeterVolumePerTickMilliliters;
          float heatingEnergy = infusionVolume * (config.temperature - config.waterTemperature) * HEATING_ENERGY_PER_ML_AND_KELVIN_WATTSECONDS * config.volumeBasedHeatingFactor;
          unsigned int heatingCyclesSet = heatingEnergy / HeatingOutputWatts * 1000 / HEATING_CYCLE_LENGTH;
          if (heatingCyclesSet > infusionHeatingCyclesIs) 
          {
            heatingRelay.setCycles(heatingCyclesSet - infusionHeatingCyclesIs, false);
            infusionHeatingCyclesIs = heatingCyclesSet;
          }
        }
        else
        {
          heatingRelay.setCycles(0, true);
        }
    }
  }

  if (waterLevelSensorPresent) 
  {
    if (waterLevelSensor.isRangeComplete())
    {
      uint8_t waterLevelSample = waterLevelSensor.readRangeResult();
      if (waterLevelAverage.addSample(waterLevelSample))
      {
        waterLevel = waterLevelAverage.average(1);
      }
      waterLevelSensor.startRange();
    }
  }

  if (!infusing && !steam)
  {
    if (abs(temperateAvg.get() - temperatureSet) < 0.5f && brewingUnitTemperateAvg.get() >= brewingUnitTemperature)
    {
      if (readyCycleCount % (READY_NOTIFICATION_INTERVAL / CycleLengthMillis) == 0)
      {
        readyMelody();
      }
      readyCycleCount++;
    }
  }

  int32_t currentScaleValue;
  unsigned long currentScaleValueTimestamp;
  bleServer.scaleValue(currentScaleValue, currentScaleValueTimestamp);
  scaleConnected = millis() - currentScaleValueTimestamp < 1000;
  if (scaleConnected)
  {
    if (millis() - lastScaleValueTimestamp >= 500)
    {
      flowWeightAvg.push(((currentScaleValue - lastScaleValue) / 10.0) / ((currentScaleValueTimestamp - lastScaleValueTimestamp) / 1000.0));
      lastScaleValue = currentScaleValue;
      lastScaleValueTimestamp = currentScaleValueTimestamp;
    }
    scaleValue = currentScaleValue;
  }

if (cycle % 25 == 0)
{
  Serial.printf("pahse length %d\n", zeroCrossDetector.phaseLengthUs());
}

  cycle++;
  
  unsigned int nextLoopTime = startupTime + cycle * CycleLengthMillis;
  unsigned int now = millis();
  if (now < nextLoopTime)
  {
    delay(nextLoopTime - now);
  }
  else
  {
    Serial.printf("Cycle %d lag %d", cycle, nextLoopTime - now);
  }
}
