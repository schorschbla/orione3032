#pragma once

#include <Arduino.h>

template<typename T> class AntiFlappingFilter
{
public:
  AntiFlappingFilter(unsigned int duration) : duration(duration), lastChange(0) {}

  T apply(T value)
  {
    if (value != current)
    {
      if (value == previous)
      {
        unsigned int time = millis();
        if (time - lastChange < duration)
        {
          return current;
        }
        lastChange = time;
      }
      previous = current;
      current = value;
    }
    return current;
  }

private:
  T current;
  T previous;
  unsigned int duration;
  unsigned int lastChange;
};
