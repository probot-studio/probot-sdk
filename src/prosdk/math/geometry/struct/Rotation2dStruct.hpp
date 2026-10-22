// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Struct<prosdk::math::Rotation2d> final {
  static constexpr std::string_view GetTypeName() { return "Rotation2d"; }
  static constexpr size_t GetSize() { return 8; }
  static constexpr std::string_view GetSchema() { return "double value"; }

  static prosdk::math::Rotation2d Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data, const prosdk::math::Rotation2d& value);
};

static_assert(prosdk::util::StructSerializable<prosdk::math::Rotation2d>);
