#include "gauge.h"

Gauge::Gauge(unsigned int x, unsigned int y, unsigned int radius, unsigned int thickness, unsigned int bgColor) :
    x(x), y(y), 
    radius(radius), thickness(thickness), 
    color(0), start(__INT_MAX__), end(__INT_MAX__), 
    dirtyStart(__INT_MAX__), dirtyEnd(__INT_MAX__),
    bgColor(bgColor)
{   
}

inline unsigned int wrapAngle(int angle)
{
    return (angle < 0 ? 360 + angle : angle) % 360;
}

void Gauge::setValue(unsigned int start, unsigned int amount)
{   
    this->start = start;
    this->end = this->start + max(1u, amount);
}

void Gauge::setColor(unsigned int color)
{
    this->color = color;
}

void Gauge::draw(TFT_eSPI &tft)
{
    if (isDirty())
    {
        if (dirtyStart != __INT_MAX__)
        {
            if (start > dirtyStart && start < dirtyEnd)
            {
                tft.drawArc(x, y, radius + thickness, radius, wrapAngle(dirtyStart - 1), wrapAngle(start + 1), bgColor, bgColor);
            }
            if (dirtyStart < end && dirtyEnd > end)
            {
                tft.drawArc(x, y, radius + thickness, radius, wrapAngle(end - 1), wrapAngle(dirtyEnd + 2), bgColor, bgColor);
            }
        }
        
        tft.drawSmoothArc(x, y, radius + thickness, radius, wrapAngle(start), wrapAngle(end), color, bgColor);

        dirtyStart = start;
        dirtyEnd = end;
        dirtyColor = color;
    }
}

bool Gauge::isDirty() const
{
    return start != dirtyStart || end != dirtyEnd || color != dirtyColor;
}

GaugeScale::GaugeScale(unsigned int x, unsigned int y, unsigned int radius, unsigned int thickness, int start, unsigned int amount, unsigned int segments, unsigned int color, unsigned int bgColor) :
    x(x), y(y), 
    radius(radius), thickness(thickness),
    start(wrapAngle(start)), amount(amount),
    segments(segments),
    color(color), bgColor(bgColor)
{
}

void GaugeScale::draw(TFT_eSPI &tft, unsigned int space)
{
    unsigned int angle = (amount - (segments - 1) * space) / segments;
    for (int i = 0; i < segments; ++i)
    {
        unsigned int segmentStart = start + i * (angle + space);
        tft.drawSmoothArc(x, y, radius + thickness, radius, segmentStart % 360, (segmentStart + angle) % 360, color, bgColor);        
    }
}

const float degRad = 0.0174532925;

void GaugeScale::drawLabel(TFT_eSPI &tft, unsigned int pos, const char *label, int offsetX, int offsetY)
{
    unsigned int angle = start + ((float)(pos) / segments * amount);
    float x = cos((angle + 90) * degRad) * radius + this->x;
    float y = sin((angle + 90) * degRad) * radius + this->y;

    tft.setTextColor(color, bgColor);
    tft.drawString(label, x + offsetX, y + offsetY);
}
