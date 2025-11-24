#pragma once

template<typename T, unsigned int N> class MedianAverage
{
public:
  MedianAverage() : pos(0) {}

  bool addSample(T sample)
  {
    samples[pos % N] = sample;
    pos++;
    return pos >= N;
  }

  T average(unsigned int count = 1)
  {
    count = std::min(count, N);
    pos = std::min(pos, N);
    if ((pos - count) % 2 == 1)
    {
      count++;
    }
    std::sort(samples, samples + pos);
    unsigned int sum = 0;
    for (int i = 0; i < count; ++i)
    {
      sum += samples[(pos - count) / 2 + i];
    }
    pos = 0;
    return (T)(sum / count);
  }

  unsigned int size() const 
  {
    return N;
  }

private:
  T samples[N];
  unsigned int pos;
};
