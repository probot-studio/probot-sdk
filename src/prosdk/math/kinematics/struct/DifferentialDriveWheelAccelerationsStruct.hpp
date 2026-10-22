// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/kinematics/DifferentialDriveWheelAccelerations.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT
    prosdk::util::Struct<prosdk::math::DifferentialDriveWheelAccelerations> {
  static constexpr std::string_view GetTypeName() {
    return "DifferentialDriveWheelAccelerations";
  }
  static constexpr size_t GetSize() { return 16; }
  static constexpr std::string_view GetSchema() {
    return "double left;double right";
  }

  static prosdk::math::DifferentialDriveWheelAccelerations Unpack(
      std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data,
                   const prosdk::math::DifferentialDriveWheelAccelerations& value);
};

static_assert(prosdk::util::StructSerializable<
              prosdk::math::DifferentialDriveWheelAccelerations>);
