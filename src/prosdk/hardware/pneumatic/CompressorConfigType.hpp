// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace prosdk {
/**
 * Compressor config type.
 */
enum class CompressorConfigType {
  /// Disabled.
  kDisabled = 0,
  /// Digital.
  kDigital = 1,
  /// Analog.
  kAnalog = 2,
  /// Hybrid.
  kHybrid = 3
};

}  // namespace wpi
