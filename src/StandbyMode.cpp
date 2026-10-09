#include "StandbyMode.h"
#include "Fonts.h"

StandbyMode::StandbyMode()
		: _screen(nullptr),
			temperatureArc(nullptr),
			temperatureLabel(nullptr),
			brewingUnitTemperatureArc(nullptr),
			brewingUnitTemperatureLabel(nullptr),
			waterLevelArc(nullptr),
			waterLevelLabel(nullptr),
			waterLevelSymbol(nullptr),
			scaleValueLabel(nullptr),
			scaleUnitLabel(nullptr)
{

}

void StandbyMode::loop(SensorData &, Actuators &, uint32_t)
{
}

void StandbyMode::updateUi()
{
}

void StandbyMode::initUi()
{
    _screen = lv_obj_create(NULL);
    temperatureArc = lv_arc_create(_screen);
    lv_obj_set_size(temperatureArc, 230, 230);
    lv_obj_set_style_arc_width(temperatureArc, 12, LV_PART_MAIN);
    lv_obj_set_style_arc_width(temperatureArc, 12, LV_PART_INDICATOR);
    lv_arc_set_rotation(temperatureArc, 145);
    lv_arc_set_bg_angles(temperatureArc, 0, 250);
    lv_obj_remove_style(temperatureArc, NULL, LV_PART_KNOB);
    lv_obj_center(temperatureArc);

    temperatureLabel = lv_label_create(_screen);
    lv_obj_set_style_text_font(temperatureLabel, &lv_font_my_40, 0);
    lv_obj_set_width(temperatureLabel, 150);
    lv_obj_set_style_text_align(temperatureLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(temperatureLabel, LV_ALIGN_CENTER, 0, -56);

    brewingUnitTemperatureArc = lv_arc_create(_screen);
    lv_obj_set_size(brewingUnitTemperatureArc, 200, 200);
    lv_obj_set_style_arc_width(brewingUnitTemperatureArc, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_width(brewingUnitTemperatureArc, 8, LV_PART_INDICATOR);
    lv_arc_set_rotation(brewingUnitTemperatureArc, 145);
    lv_arc_set_bg_angles(brewingUnitTemperatureArc, 0, 150);
    lv_obj_remove_style(brewingUnitTemperatureArc, NULL, LV_PART_KNOB);
    lv_obj_center(brewingUnitTemperatureArc);

    waterLevelArc = lv_arc_create(_screen);
    lv_obj_set_size(waterLevelArc, 200, 200);
    lv_obj_set_style_arc_width(waterLevelArc, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_width(waterLevelArc, 8, LV_PART_INDICATOR);
    lv_arc_set_rotation(waterLevelArc, 305);
    lv_arc_set_bg_angles(waterLevelArc, 0, 90);
    lv_obj_remove_style(waterLevelArc, NULL, LV_PART_KNOB);
    lv_obj_center(waterLevelArc);

    brewingUnitTemperatureLabel = lv_label_create(_screen);
    lv_obj_set_style_text_font(brewingUnitTemperatureLabel, &lv_font_my_32, 0);
    lv_obj_set_width(brewingUnitTemperatureLabel, 160);
    lv_obj_set_style_text_align(brewingUnitTemperatureLabel, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(brewingUnitTemperatureLabel, LV_ALIGN_CENTER, 0, -23);

    waterLevelSymbol = lv_label_create(_screen);
    lv_obj_set_style_text_font(waterLevelSymbol, &lv_font_my_20, 0);
    lv_obj_set_width(waterLevelSymbol, 160);
    lv_obj_set_style_text_align(waterLevelSymbol, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(waterLevelSymbol, LV_ALIGN_CENTER, 0, -23);
    lv_label_set_text_fmt(waterLevelSymbol, "\xEF\x81\x83");

    waterLevelLabel = lv_label_create(_screen);
    lv_obj_set_style_text_font(waterLevelLabel, &lv_font_my_32, 0);
    lv_obj_set_width(waterLevelLabel, 126);
    lv_obj_set_style_text_align(waterLevelLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(waterLevelLabel, LV_ALIGN_CENTER, 0, -23);

    scaleValueLabel = lv_label_create(_screen);
    lv_obj_set_style_text_font(scaleValueLabel, &lv_font_my_68, 0);
    lv_obj_set_width(scaleValueLabel, 230);
    lv_obj_set_style_text_align(scaleValueLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(scaleValueLabel, LV_ALIGN_CENTER, 0, 32);

    scaleUnitLabel = lv_label_create(_screen);
    lv_obj_set_style_text_font(scaleUnitLabel, &lv_font_my_20, 0);
    lv_obj_set_width(scaleUnitLabel, 230);
    lv_obj_set_style_text_align(scaleUnitLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(scaleUnitLabel, LV_ALIGN_CENTER, 0, 70);
    lv_label_set_text_fmt(scaleUnitLabel, "Gramm");
}

bool StandbyMode::displaySplash()
{
	return false;
}

lv_obj_t *StandbyMode::screen()
{
	return _screen;
}
