#include <Arduino.h>
#include "PulseCounter.h"

PulseCounter::PulseCounter(uint8_t pin) : pin(pin), pcntUnit(PCNT_UNIT_MAX), overflowCounter(0)
{
}

PulseCounter::~PulseCounter()
{
    end();
}

IRAM_ATTR void PulseCounter::onInterruptArg(void *arg)
{
    static_cast<PulseCounter*>(arg)->onInterrupt();
}

int PulseCounter::begin()
{
    int ret = 0;

    pcnt_config_t pcntConfig = {
        .pulse_gpio_num = pin,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = INT16_MAX,
        .channel = PCNT_CHANNEL_0,
    };

    pinMode(pin, INPUT_PULLDOWN);

    for (int i = PCNT_UNIT_0; i < PCNT_UNIT_MAX; ++i)
    {
        pcntConfig.unit = static_cast<pcnt_unit_t>(i);
        ret = pcnt_unit_config(&pcntConfig);
        if (ret == ESP_OK)
        {
            ret = pcnt_event_enable(pcntConfig.unit, PCNT_EVT_H_LIM);
            if (ret == ESP_OK)
            {
                ret = pcnt_isr_handler_add(pcntConfig.unit, &onInterruptArg, this);
                if (ret == ESP_OK)
                {
                    pcntUnit = pcntConfig.unit;
                    reset();
                }
            }
            break;
        }
        else if (ret != ESP_ERR_INVALID_STATE)
        {
            break;
        }
    }
    
    return ret;
}

void PulseCounter::end()
{
}

IRAM_ATTR void PulseCounter::onInterrupt()
{
    overflowCounter++;
}

uint32_t PulseCounter::ticks()
{
    int16_t ticks;
    uint32_t overflowCounter;
    do
    {
        overflowCounter = this->overflowCounter;
        pcnt_get_counter_value(pcntUnit, &ticks);
    } while (overflowCounter != this->overflowCounter);

    return overflowCounter * INT16_MAX + ticks;
}

void PulseCounter::reset()
{
    pcnt_counter_clear(pcntUnit);
    overflowCounter = 0;
}
