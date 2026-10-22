// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <string_view>

#include "prosdk/system/Tracer.hpp"

namespace prosdk::util {
class raw_ostream;
}  // namespace prosdk::util

namespace prosdk {
/**
 * A class for keeping track of how much time it takes for different
 * parts of code to execute. This class uses RAII, meaning you simply
 * need to create an instance at the top of the block you are timing. After the
 * block finishes execution (i.e. when the ScopedTracer instance gets
 * destroyed), the epoch is printed to the provided prosdk::util::raw_ostream.
 */
class ScopedTracer {
 public:
  /**
   * Constructs a ScopedTracer instance.
   *
   * @param name The name of the epoch.
   * @param os A reference to the prosdk::util::raw_ostream to print data to.
   */
  ScopedTracer(std::string_view name, prosdk::util::raw_ostream& os);
  ~ScopedTracer();

  ScopedTracer(const ScopedTracer&) = delete;
  ScopedTracer& operator=(const ScopedTracer&) = delete;

 private:
  Tracer m_tracer;
  std::string m_name;
  prosdk::util::raw_ostream& m_os;
};
}  // namespace wpi
