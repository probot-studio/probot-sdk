// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/linalg/struct/MatrixStruct.hpp"
#include "prosdk/math/system/LinearSystem.hpp"
#include "prosdk/util/ct_string.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <int States, int Inputs, int Outputs>
struct prosdk::util::Struct<prosdk::math::LinearSystem<States, Inputs, Outputs>> {
  static constexpr ct_string TYPE_NAME = prosdk::util::Concat(
      "LinearSystem__"_ct_string, prosdk::util::NumToCtString<States>(),
      "_"_ct_string, prosdk::util::NumToCtString<Inputs>(), "_"_ct_string,
      prosdk::util::NumToCtString<Outputs>());
  static constexpr std::string_view GetTypeName() { return TYPE_NAME; }
  static constexpr size_t GetSize() {
    return prosdk::util::Struct<prosdk::math::Matrixd<States, States>>::GetSize() +
           prosdk::util::Struct<prosdk::math::Matrixd<States, Inputs>>::GetSize() +
           prosdk::util::Struct<prosdk::math::Matrixd<Outputs, States>>::GetSize() +
           prosdk::util::Struct<prosdk::math::Matrixd<Outputs, Inputs>>::GetSize();
  }
  static constexpr ct_string SCHEMA = prosdk::util::Concat(
      prosdk::util::Struct<prosdk::math::Matrixd<States, States>>::TYPE_NAME,
      " a;"_ct_string,
      prosdk::util::Struct<prosdk::math::Matrixd<States, Inputs>>::TYPE_NAME,
      " b;"_ct_string,
      prosdk::util::Struct<prosdk::math::Matrixd<Outputs, States>>::TYPE_NAME,
      " c;"_ct_string,
      prosdk::util::Struct<prosdk::math::Matrixd<Outputs, Inputs>>::TYPE_NAME,
      " d"_ct_string);
  static constexpr std::string_view GetSchema() { return SCHEMA; }

  static prosdk::math::LinearSystem<States, Inputs, Outputs> Unpack(
      std::span<const uint8_t> data) {
    constexpr size_t A_OFF = 0;
    constexpr size_t B_OFF =
        A_OFF + prosdk::util::GetStructSize<prosdk::math::Matrixd<States, States>>();
    constexpr size_t C_OFF =
        B_OFF + prosdk::util::GetStructSize<prosdk::math::Matrixd<States, Inputs>>();
    constexpr size_t D_OFF =
        C_OFF + prosdk::util::GetStructSize<prosdk::math::Matrixd<Outputs, States>>();
    return prosdk::math::LinearSystem<States, Inputs, Outputs>{
        prosdk::util::UnpackStruct<prosdk::math::Matrixd<States, States>, A_OFF>(
            data),
        prosdk::util::UnpackStruct<prosdk::math::Matrixd<States, Inputs>, B_OFF>(
            data),
        prosdk::util::UnpackStruct<prosdk::math::Matrixd<Outputs, States>, C_OFF>(
            data),
        prosdk::util::UnpackStruct<prosdk::math::Matrixd<Outputs, Inputs>, D_OFF>(
            data)};
  }

  static void Pack(
      std::span<uint8_t> data,
      const prosdk::math::LinearSystem<States, Inputs, Outputs>& value) {
    constexpr size_t A_OFF = 0;
    constexpr size_t B_OFF =
        A_OFF + prosdk::util::GetStructSize<prosdk::math::Matrixd<States, States>>();
    constexpr size_t C_OFF =
        B_OFF + prosdk::util::GetStructSize<prosdk::math::Matrixd<States, Inputs>>();
    constexpr size_t D_OFF =
        C_OFF + prosdk::util::GetStructSize<prosdk::math::Matrixd<Outputs, States>>();
    prosdk::util::PackStruct<A_OFF>(data, value.A());
    prosdk::util::PackStruct<B_OFF>(data, value.B());
    prosdk::util::PackStruct<C_OFF>(data, value.C());
    prosdk::util::PackStruct<D_OFF>(data, value.D());
  }

  static void ForEachNested(
      std::invocable<std::string_view, std::string_view> auto fn) {
    prosdk::util::ForEachStructSchema<prosdk::math::Matrixd<States, States>>(fn);
    prosdk::util::ForEachStructSchema<prosdk::math::Matrixd<States, Inputs>>(fn);
    prosdk::util::ForEachStructSchema<prosdk::math::Matrixd<Outputs, States>>(fn);
    prosdk::util::ForEachStructSchema<prosdk::math::Matrixd<Outputs, Inputs>>(fn);
  }
};

static_assert(prosdk::util::StructSerializable<prosdk::math::LinearSystem<4, 3, 2>>);
static_assert(prosdk::util::HasNestedStruct<prosdk::math::LinearSystem<4, 3, 2>>);
static_assert(prosdk::util::StructSerializable<prosdk::math::LinearSystem<2, 3, 4>>);
static_assert(prosdk::util::HasNestedStruct<prosdk::math::LinearSystem<2, 3, 4>>);
