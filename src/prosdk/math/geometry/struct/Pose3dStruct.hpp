// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/geometry/Pose3d.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Struct<prosdk::math::Pose3d> final {
  static constexpr std::string_view GetTypeName() { return "Pose3d"; }
  static constexpr size_t GetSize() {
    return prosdk::util::GetStructSize<prosdk::math::Translation3d>() +
           prosdk::util::GetStructSize<prosdk::math::Rotation3d>();
  }
  static constexpr std::string_view GetSchema() {
    return "Translation3d translation;Rotation3d rotation";
  }

  static prosdk::math::Pose3d Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data, const prosdk::math::Pose3d& value);
  static void ForEachNested(
      std::invocable<std::string_view, std::string_view> auto fn) {
    prosdk::util::ForEachStructSchema<prosdk::math::Translation3d>(fn);
    prosdk::util::ForEachStructSchema<prosdk::math::Rotation3d>(fn);
  }
};

static_assert(prosdk::util::StructSerializable<prosdk::math::Pose3d>);
static_assert(prosdk::util::HasNestedStruct<prosdk::math::Pose3d>);
