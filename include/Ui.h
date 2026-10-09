#pragma once

#include <lvgl.h>

class Ui
{
public:
    virtual void initUi() = 0;
    virtual bool displaySplash() = 0;
    virtual lv_obj_t* screen() = 0;
};
