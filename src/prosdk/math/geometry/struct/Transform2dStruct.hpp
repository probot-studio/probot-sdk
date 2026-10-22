// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/geometry/Transform2d.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Struct<prosdk::math::Transform2d> final {
  static constexpr std::string_view GetTypeName() { return "Transform2d"; }
  static constexpr size_t GetSize() {
    return prosdk::util::GetStructSize<prosdk::math::Translation2d>() +
           prosdk::util::GetStructSize<prosdk::math::Rotation2d>();
  }
  static constexpr std::string_view GetSchema() {
    return "Translation2d translation;Rotation2d rotation";
  }

  static prosdk::math::Transform2d Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data,
                   const prosdk::math::Transform2d& value);
  static void ForEachNested(
      std::invocable<std::string_view, std::string_view> auto fn) {
    prosdk::util::ForEachStructSchema<prosdk::math::Translation2d>(fn);
    prosdk::util::ForEachStructSchema<prosdk::math::Rotation2d>(fn);
  }
};

static_assert(prosdk::util::StructSerializable<prosdk::math::Transform2d>);
static_assert(prosdk::util::HasNestedStruct<prosdk::math::Transform2d>);
