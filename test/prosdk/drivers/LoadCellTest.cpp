// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/adc/LoadCell.hpp"

#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {
// A source that hands out queued samples, then nothing.
struct FakeSource {
  std::optional<int32_t> operator()() {
    if (samples.empty()) {
      return std::nullopt;
    }
    auto s = samples.front();
    samples.pop_front();
    return s;
  }
  std::deque<std::optional<int32_t>> samples;
};
}  // namespace

TEST_CASE("LoadCell tares and scales", "[loadcell]") {
  FakeSource source;
  LoadCell cell{[&source] { return source(); }};
  CHECK_FALSE(cell.HasData());
  CHECK_FALSE(cell.Update());
  CHECK(cell.GetMass().value() == Catch::Approx(0.0).margin(1e-12));

  // tare over 4 samples; a poll without a new sample does not count
  cell.Tare(4);
  CHECK(cell.IsTaring());
  source.samples = {1000, std::nullopt, 1003, 997, 1002};
  for (int i = 0; i < 4; ++i) {
    cell.Update();
  }
  CHECK(cell.IsTaring());
  CHECK(cell.GetOffset() == Catch::Approx(0.0).margin(1e-12));  // old offset until done
  CHECK(cell.GetMass().value() == Catch::Approx(997.0));          // scale 1: counts
  CHECK(cell.Update());
  CHECK_FALSE(cell.IsTaring());
  CHECK(cell.GetOffset() == Catch::Approx(1000.5));
  CHECK(cell.GetCounts() == Catch::Approx(1.5));

  // calibration: 2.5 kg gives 50000 counts above the offset
  source.samples = {51000};
  CHECK(cell.Update());
  CHECK(cell.GetCounts() == Catch::Approx(49999.5));
  cell.SetScale(20000.0);
  CHECK(cell.GetScale() == Catch::Approx(20000.0));
  CHECK(cell.GetMass().value() == Catch::Approx(49999.5 / 20000.0));
  source.samples = {1000 - 20000};
  cell.Update();
  CHECK(cell.GetMass().value() == Catch::Approx(-20000.5 / 20000.0));

  // a cell mounted the other way round
  cell.SetScale(-20000.0);
  CHECK(cell.GetMass().value() == Catch::Approx(20000.5 / 20000.0));
  CHECK(cell.HasData());
}

TEST_CASE("LoadCell argument errors", "[loadcell]") {
  LoadCell cell{[] { return std::optional<int32_t>{}; }};
  CHECK_THROWS_AS(cell.Tare(0), RuntimeError);
  CHECK_THROWS_AS(cell.SetScale(0.0), RuntimeError);
  CHECK_THROWS_AS(cell.SetScale(std::nan("")), RuntimeError);
  CHECK_THROWS_AS(cell.SetScale(INFINITY), RuntimeError);
  CHECK(cell.GetScale() == Catch::Approx(1.0));
  CHECK_FALSE(cell.IsTaring());
}
