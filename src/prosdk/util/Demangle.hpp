// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <string_view>
#include <typeinfo>

#include "prosdk/util/type_name.hpp"

namespace prosdk::util {

/**
 * Demangle a C++ symbol.
 *
 * @param mangledSymbol the mangled symbol.
 * @return The demangled symbol, or mangledSymbol if demangling fails.
 */
std::string Demangle(std::string_view mangledSymbol);

/**
 * Returns the type name of an object
 *
 * prosdk: built without RTTI, this is the static type T (from __PRETTY_FUNCTION__), not the
 * dynamic type of the object. Classes that need their most-derived name get it from their CRTP
 * helper (for example CommandHelper) or SetName().
 *
 * @param type The object
 */
template <typename T>
std::string GetTypeName(const T& type) {
#if defined(__GXX_RTTI) || defined(__cpp_rtti)
  return Demangle(typeid(type).name());
#else
  (void)type;
  return std::string{GetTypeName<T>()};
#endif
}

}  // namespace prosdk::util
