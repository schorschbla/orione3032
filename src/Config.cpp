#include "Config.h"

#include <ArduinoJson.h>

Config::Config(const ConfigValues &defaultValues)
    : values(defaultValues)
{
}

double Config::standbyTemperatureDegCelsius() const
{
    return values.standbyTemperatureDegCelsius;
}

double Config::standbyPidP() const
{
    return values.standbyPidP;
}

double Config::standbyPidI() const
{
    return values.standbyPidI;
}

double Config::standbyPidD() const
{
    return values.standbyPidD;
}

bool Config::load(Stream &s)
{
    JsonDocument document;
    if (deserializeJson(document, s) || !document.is<JsonObject>())
    {
        return false;
    }

    JsonObjectConst standby = document["standby"].as<JsonObjectConst>();
    if (standby.isNull())
    {
        return false;
    }

    ConfigValues loadedValues = values;
    loadedValues.standbyTemperatureDegCelsius =
        standby["temperatureDegCelsius"] | values.standbyTemperatureDegCelsius;
    loadedValues.standbyPidP = standby["pidP"] | values.standbyPidP;
    loadedValues.standbyPidI = standby["pidI"] | values.standbyPidI;
    loadedValues.standbyPidD = standby["pidD"] | values.standbyPidD;

    values = loadedValues;
    return true;
}

bool Config::persist(Stream &s)
{
    JsonDocument document;
    JsonObject standby = document["standby"].to<JsonObject>();
    standby["temperatureDegCelsius"] = values.standbyTemperatureDegCelsius;
    standby["pidP"] = values.standbyPidP;
    standby["pidI"] = values.standbyPidI;
    standby["pidD"] = values.standbyPidD;

    s.clearWriteError();
    return serializeJson(document, s) > 0 && s.getWriteError() == 0;
}
