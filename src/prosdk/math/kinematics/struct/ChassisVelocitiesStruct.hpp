// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/kinematics/ChassisVelocities.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Struct<prosdk::math::ChassisVelocities> {
  static constexpr std::string_view GetTypeName() {
    return "ChassisVelocities";
  }
  static constexpr size_t GetSize() { return 24; }
  static constexpr std::string_view GetSchema() {
    return "double vx;double vy;double omega";
  }

  static prosdk::math::ChassisVelocities Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data,
                   const prosdk::math::ChassisVelocities& value);
};

static_assert(prosdk::util::StructSerializable<prosdk::math::ChassisVelocities>);
