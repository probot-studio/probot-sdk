// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// prosdk::cxx::invoke_r: C++23 std::invoke_r ([func.invoke]) for C++20.

#pragma once

#include <functional>
#include <type_traits>
#include <utility>

namespace prosdk::cxx {

template <class R, class Fn, class... Args>
  requires std::is_invocable_r_v<R, Fn, Args...>
constexpr R invoke_r(Fn&& f, Args&&... args) noexcept(std::is_nothrow_invocable_r_v<R, Fn, Args...>) {
  if constexpr (std::is_void_v<R>) {
    std::invoke(std::forward<Fn>(f), std::forward<Args>(args)...);
  } else {
    return std::invoke(std::forward<Fn>(f), std::forward<Args>(args)...);
  }
}

}  // namespace prosdk::cxx
