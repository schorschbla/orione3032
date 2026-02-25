#pragma once

class ColorScale
{
public:
    ColorScale(const unsigned int *rgb, const float *weights, unsigned char length);
    unsigned int getRgb(float value) const;

private:
    const unsigned int *rgb;
    const float *weights;
    unsigned char length;    
};