// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// prosdk::cxx::to_underlying and prosdk::cxx::unreachable: C++23 [utility.underlying] and
// [utility.unreachable] for C++20.

#pragma once

#include <type_traits>

namespace prosdk::cxx {

template <class Enum>
  requires std::is_enum_v<Enum>
constexpr std::underlying_type_t<Enum> to_underlying(Enum e) noexcept {
  return static_cast<std::underlying_type_t<Enum>>(e);
}

[[noreturn]] inline void unreachable() {
  __builtin_unreachable();
}

}  // namespace prosdk::cxx
