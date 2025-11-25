#pragma once

#include <Arduino.h>

template<typename T, unsigned int S> class AntiFlappingFilter
{
public:
  AntiFlappingFilter(unsigned int deadline) : deadline(deadline), pos(0) {}

  T apply(T value)
  {
    unsigned int time = millis();
    for (int i = 0; i < S; ++i)
    {
      if (records[i].value == value && time - records[i].time < deadline)
      {
        return records[pos].value;
      }
    }
    pos = (pos + 1) % S;
    records[pos].value = value;
    records[pos].time = time;
    return value;
  }

private:
  typedef struct 
  {
    T value;
    unsigned int time;
  } Record;

  Record records[S];
  unsigned int pos;
  unsigned int deadline;
};
