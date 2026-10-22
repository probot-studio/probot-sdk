// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/system/ScopedTracer.hpp"

#include <string_view>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "prosdk/simulation/SimHooks.hpp"
#include "prosdk/util/SmallString.hpp"
#include "prosdk/util/StringExtras.hpp"
#include "prosdk/util/raw_ostream.hpp"

TEST_CASE("ScopedTracerTest Timing", "[wpilibc]") {
  prosdk::util::SmallString<128> buf;
  prosdk::util::raw_svector_ostream os(buf);

  prosdk::sim::PauseTiming();
  {
    prosdk::ScopedTracer tracer("timing_test", os);
    prosdk::sim::StepTiming(1.5_s);
  }
  prosdk::sim::ResumeTiming();

  std::string_view out = os.str();
  CHECK(prosdk::util::starts_with(out, "\ttiming_test: 1.5"));
}
