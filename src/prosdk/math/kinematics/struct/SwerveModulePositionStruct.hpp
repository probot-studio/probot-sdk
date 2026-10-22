// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/kinematics/SwerveModulePosition.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Struct<prosdk::math::SwerveModulePosition> {
  static constexpr std::string_view GetTypeName() {
    return "SwerveModulePosition";
  }
  static constexpr size_t GetSize() {
    return 8 + prosdk::util::GetStructSize<prosdk::math::Rotation2d>();
  }
  static constexpr std::string_view GetSchema() {
    return "double distance;Rotation2d angle";
  }

  static prosdk::math::SwerveModulePosition Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data,
                   const prosdk::math::SwerveModulePosition& value);
  static void ForEachNested(
      std::invocable<std::string_view, std::string_view> auto fn) {
    prosdk::util::ForEachStructSchema<prosdk::math::Rotation2d>(fn);
  }
};

static_assert(prosdk::util::StructSerializable<prosdk::math::SwerveModulePosition>);
static_assert(prosdk::util::HasNestedStruct<prosdk::math::SwerveModulePosition>);
