// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/util/StackTrace.hpp"

#include "prosdk/platform/config.hpp"

#if PROSDK_TARGET_HW
#include <cinttypes>
#include <cstdio>

#include "prosdk/platform/Backtrace.hpp"
#elif defined(__cpp_lib_stacktrace)
#include <stacktrace>
#elif !defined(_WIN32) && !defined(__EMSCRIPTEN__)
#include <execinfo.h>
#endif

#include <string>

#include "prosdk/util/Demangle.hpp"
#include "prosdk/util/SmallString.hpp"
#include "prosdk/util/StringExtras.hpp"
#include "prosdk/util/raw_ostream.hpp"

namespace prosdk::util {

std::string GetStackTraceDefault(int offset) {
  prosdk::util::SmallString<1024> buf;
  prosdk::util::raw_svector_ostream trace(buf);

#if PROSDK_TARGET_HW
  // ESP32: return addresses only; decode with xtensa-esp32s3-elf-addr2line -pfiaC -e <sketch.elf>
  std::uintptr_t pcs[32];
  size_t n = prosdk::platform::CaptureBacktrace(pcs, 32);
  for (size_t i = offset; i < n; ++i) {
    if (i == 0 || pcs[i] != pcs[i - 1]) {
      char addr[24];
      std::snprintf(addr, sizeof(addr), "0x%08" PRIxPTR, pcs[i]);
      trace << "\tat " << addr << '\n';
    }
  }

  return std::string{trace.str()};
#elif defined(__cpp_lib_stacktrace)
  auto stackTrace = std::stacktrace::current();

  for (size_t i = offset; i < stackTrace.size(); ++i) {
    // Only print recursive functions once in a row
    if (i == 0 || stackTrace[i] != stackTrace[i - 1]) {
      trace << "\tat " << std::to_string(stackTrace[i]) << '\n';
    }
  }

  return std::string{trace.str()};
#elif !defined(_WIN32) && !defined(__ANDROID__) && !defined(__EMSCRIPTEN__)
  void* stackTrace[128];
  int stackSize = backtrace(stackTrace, 128);
  char** mangledSymbols = backtrace_symbols(stackTrace, stackSize);

  for (int i = offset; i < stackSize; ++i) {
    // Only print recursive functions once in a row
    if (i == 0 || stackTrace[i] != stackTrace[i - 1]) {
      // Extract just function name from "pathToExe(functionName+offset)"
      std::string_view sym = split(mangledSymbols[i], '(').second;
      std::string_view offset;
      std::tie(sym, offset) = split(sym, '+');
      std::string_view addr;
      std::tie(offset, addr) = split(offset, ')');

      trace << "\tat " << Demangle(sym) << " + " << offset << addr << '\n';
    }
  }

  std::free(mangledSymbols);

  return std::string{trace.str()};
#else
  // backtrace_symbols not supported
  return "";
#endif
}

}  // namespace prosdk::util
