// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/kinematics/ChassisAccelerations.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Struct<prosdk::math::ChassisAccelerations> {
  static constexpr std::string_view GetTypeName() {
    return "ChassisAccelerations";
  }
  static constexpr size_t GetSize() { return 24; }
  static constexpr std::string_view GetSchema() {
    return "double ax;double ay;double alpha";
  }

  static prosdk::math::ChassisAccelerations Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data,
                   const prosdk::math::ChassisAccelerations& value);
};

static_assert(prosdk::util::StructSerializable<prosdk::math::ChassisAccelerations>);
