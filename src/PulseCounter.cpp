#include <Arduino.h>
#include "PulseCounter.h"

PulseCounter::PulseCounter(uint8_t pin) : pin(pin), pcntUnit(nullptr), pcntChannel(nullptr)
{
}

PulseCounter::~PulseCounter()
{
    end();
}

int PulseCounter::begin()
{
    esp_err_t ret;

    pcnt_unit_config_t unit_config = 
    {
        .low_limit = 0,
        .high_limit = UINT16_MAX,
        .flags = 
        {
            .accum_count = 1,
        }
    };

    ret = pcnt_new_unit(&unit_config, &pcntUnit);
    if (ret != ESP_OK)
    {
        return ret;
    }

    pcnt_chan_config_t channelConfig = 
    {
        .edge_gpio_num = pin,
        .level_gpio_num = -1,
    };

    ret = pcnt_new_channel(pcntUnit, &channelConfig, &pcntChannel);
    if (ret != ESP_OK)
    {
        return ret;
    }
 
    ret = pcnt_channel_set_edge_action(pcntChannel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = pcnt_unit_enable(pcntUnit);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = pcnt_unit_clear_count(pcntUnit);
    if (ret != ESP_OK)
    {
        return ret;
    }  

    ret = pcnt_unit_start(pcntUnit);
     if (ret != ESP_OK)
    {
        return ret;
    }

    return 0;
}

void PulseCounter::end()
{
    if (pcntChannel != nullptr)
    {
        pcnt_del_channel(pcntChannel);
        pcntChannel = nullptr;
    }

    if (pcntUnit != nullptr)
    {
        pcnt_unit_disable(pcntUnit);
        pcnt_del_unit(pcntUnit);
        pcntUnit = nullptr;
    }
}

uint32_t PulseCounter::ticks()
{
    esp_err_t ret;
    int value;
    ret = pcnt_unit_get_count(pcntUnit, &value);
    return ret == ESP_OK ? value : 0;
}

void PulseCounter::reset()
{
    pcnt_unit_clear_count(pcntUnit);
}
