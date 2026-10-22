// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/geometry/Pose3d.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::util {
class json;
}  // namespace prosdk::util

namespace prosdk::fields {

/**
 * Represents a field tag's metadata.
 */
struct WPILIB_DLLEXPORT FieldTag final {
  /// The tag's ID.
  int ID;

  /// The tag's pose.
  prosdk::math::Pose3d pose;

  bool operator==(const FieldTag&) const = default;
};

WPILIB_DLLEXPORT
void to_json(prosdk::util::json& json, const FieldTag& tag);

WPILIB_DLLEXPORT
void from_json(const prosdk::util::json& json, FieldTag& tag);

}  // namespace prosdk::fields
