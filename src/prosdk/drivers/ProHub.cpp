// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/drivers/ProHub.hpp"

#include "prosdk/platform/HalConfig.hpp"

namespace prosdk::prohub {

bool ConfigurePins() {
  bool ok = platform::SetI2CPins(0, kI2CSda, kI2CScl, kI2CFrequency);
  return platform::SetSerialPins(0, kEncoderRx, kEncoderTx) && ok;
}

}  // namespace prosdk::prohub
