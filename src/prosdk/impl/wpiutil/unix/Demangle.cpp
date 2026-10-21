// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/util/Demangle.hpp"

#include <cstdio>
#include <cxxabi.h>
#include <string>

#include "prosdk/util/SmallString.hpp"

namespace prosdk::util {

std::string Demangle(std::string_view mangledSymbol) {
  SmallString<128> buf{mangledSymbol};
  size_t length;
  int status;  // prosdk: int32_t is long on Xtensa; __cxa_demangle takes int*

  char* symbol = abi::__cxa_demangle(buf.c_str(), nullptr, &length, &status);
  if (status == 0) {
    std::string rv{symbol};
    std::free(symbol);
    return rv;
  }

  // If everything else failed, just return the mangled symbol
  return std::string{mangledSymbol};
}

}  // namespace prosdk::util
