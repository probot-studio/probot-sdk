// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/kinematics/SwerveDriveKinematics.hpp"
#include "prosdk/util/ct_string.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <size_t NumModules>
struct prosdk::util::Struct<prosdk::math::SwerveDriveKinematics<NumModules>> {
  static constexpr ct_string TYPE_NAME =
      prosdk::util::Concat("SwerveDriveKinematics__"_ct_string,
                        prosdk::util::NumToCtString<NumModules>());
  static constexpr std::string_view GetTypeName() { return TYPE_NAME; }
  static constexpr size_t GetSize() {
    return NumModules * prosdk::util::Struct<prosdk::math::Translation2d>::GetSize();
  }
  static constexpr ct_string SCHEMA =
      prosdk::util::Concat("Translation2d modules["_ct_string,
                        prosdk::util::NumToCtString<NumModules>(), "]"_ct_string);
  static constexpr std::string_view GetSchema() { return SCHEMA; }

  static prosdk::math::SwerveDriveKinematics<NumModules> Unpack(
      std::span<const uint8_t> data) {
    constexpr size_t MODULES_OFF = 0;
    return prosdk::math::SwerveDriveKinematics<NumModules>{
        prosdk::util::UnpackStructArray<prosdk::math::Translation2d, MODULES_OFF,
                                     NumModules>(data)};
  }

  static void Pack(std::span<uint8_t> data,
                   const prosdk::math::SwerveDriveKinematics<NumModules>& value) {
    constexpr size_t MODULES_OFF = 0;
    prosdk::util::PackStructArray<MODULES_OFF, NumModules>(data,
                                                        value.GetModules());
  }

  static void ForEachNested(
      std::invocable<std::string_view, std::string_view> auto fn) {
    prosdk::util::ForEachStructSchema<prosdk::math::Translation2d>(fn);
  }
};

static_assert(
    prosdk::util::StructSerializable<prosdk::math::SwerveDriveKinematics<4>>);
static_assert(prosdk::util::HasNestedStruct<prosdk::math::SwerveDriveKinematics<4>>);
static_assert(
    prosdk::util::StructSerializable<prosdk::math::SwerveDriveKinematics<3>>);
static_assert(prosdk::util::HasNestedStruct<prosdk::math::SwerveDriveKinematics<3>>);
