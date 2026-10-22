// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Struct<prosdk::math::Rotation3d> final {
  static constexpr std::string_view GetTypeName() { return "Rotation3d"; }
  static constexpr size_t GetSize() {
    return prosdk::util::GetStructSize<prosdk::math::Quaternion>();
  }
  static constexpr std::string_view GetSchema() { return "Quaternion q"; }

  static prosdk::math::Rotation3d Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data, const prosdk::math::Rotation3d& value);
  static void ForEachNested(
      std::invocable<std::string_view, std::string_view> auto fn) {
    prosdk::util::ForEachStructSchema<prosdk::math::Quaternion>(fn);
  }
};

static_assert(prosdk::util::StructSerializable<prosdk::math::Rotation3d>);
static_assert(prosdk::util::HasNestedStruct<prosdk::math::Rotation3d>);
