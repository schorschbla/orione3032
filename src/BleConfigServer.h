#pragma once

#include <functional>
#include "Qm3032Config.h"

class BleConfigServer
{
public:
  using WriteConfigCallback = std::function<bool(const Qm3032Config &)>;

  BleConfigServer();
  void begin(Qm3032Config *config, WriteConfigCallback writeCallback);

private:
  Qm3032Config *config_;
  WriteConfigCallback writeCallback_;
};
