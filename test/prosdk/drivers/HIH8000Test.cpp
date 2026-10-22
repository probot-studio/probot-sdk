// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/environment/HIH8000.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/simulation/SimHooks.hpp"

using namespace prosdk;

namespace {
// The chip has no register pointer: every Data Fetch returns the same 4 bytes, kept in regs[0..3].
test::FakeI2CBus::Device& AddChip(test::FakeI2CBus& bus) {
  auto& dev = bus.Add(0x27);
  dev.onRead = [&dev](std::span<uint8_t> buf) {
    for (std::size_t i = 0; i < buf.size(); ++i) {
      buf[i] = static_cast<uint8_t>(i < 4 ? dev.regs[i] : 0);
    }
  };
  return dev;
}

// A Data Fetch answer (technical note 009061-2-EN Figure 4): status in bits 7:6 of the first
// byte, 14-bit humidity, 14-bit temperature left-aligned in the last two bytes.
void SetFetch(test::FakeI2CBus::Device& dev, int status, int humidity, int temperature) {
  dev.regs[0] = static_cast<uint8_t>((status << 6) | (humidity >> 8));
  dev.regs[1] = static_cast<uint8_t>(humidity & 0xFF);
  dev.regs[2] = static_cast<uint8_t>(temperature >> 6);
  dev.regs[3] = static_cast<uint8_t>((temperature & 0x3F) << 2);
}
}  // namespace

TEST_CASE("HIH8000 requests, fetches and converts a measurement", "[hih8000]") {
  sim::PauseTiming();
  test::FakeI2CBus bus;
  auto& dev = AddChip(bus);
  SetFetch(dev, 0, 8191, 6620);
  HIH8000 sensor{I2C::Port::PORT_0};
  CHECK(dev.writes.empty());
  CHECK(sensor.IsStale());

  sensor.Update();  // Measurement Request: an empty write
  CHECK(sensor.IsConnected());
  REQUIRE(dev.writes.size() == 1);
  CHECK(dev.writes[0].empty());
  sensor.Update();  // still measuring: no transfer
  CHECK(dev.reads == 0);

  sim::StepTiming(50_ms);
  sensor.Update();  // fetch, then the next request
  CHECK(dev.reads == 1);
  CHECK(dev.writes.size() == 2);
  CHECK(sensor.GetHumidity() == Catch::Approx(50.0));  // 8191 / 16382
  CHECK(sensor.GetTemperature().value() == Catch::Approx(6620 * 165.0 / 16382 - 40));  // 26.68 C
  CHECK_FALSE(sensor.IsStale());

  // stale data (status 01): values stay, no new request until 0.5 s without data
  SetFetch(dev, 1, 0, 0);
  sim::StepTiming(50_ms);
  sensor.Update();
  CHECK(dev.reads == 2);
  CHECK(dev.writes.size() == 2);
  CHECK(sensor.GetHumidity() == Catch::Approx(50.0));
  sim::StepTiming(500_ms);
  sensor.Update();
  CHECK(dev.writes.size() == 2);
  sensor.Update();  // request again
  CHECK(dev.writes.size() == 3);
  sim::StepTiming(500_ms);
  CHECK(sensor.IsStale());  // last valid sample more than 1 s old

  // command mode (status 10) is not data either: request again
  SetFetch(dev, 2, 100, 100);
  sensor.Update();
  sensor.Update();
  CHECK(dev.writes.size() == 4);
  CHECK(sensor.GetHumidity() == Catch::Approx(50.0));

  SetFetch(dev, 0, 0, 16382);  // 0 %RH, 125 C
  sim::StepTiming(50_ms);
  sensor.Update();
  CHECK(sensor.GetHumidity() == Catch::Approx(0.0).margin(1e-12));
  CHECK(sensor.GetTemperature().value() == Catch::Approx(125.0));
  CHECK_FALSE(sensor.IsStale());
  sim::ResumeTiming();
}

TEST_CASE("HIH8000 reports a missing chip and recovers", "[hih8000]") {
  sim::PauseTiming();
  test::FakeI2CBus bus;
  HIH8000 sensor{I2C::Port::PORT_0};
  sensor.Update();
  CHECK_FALSE(sensor.IsConnected());
  CHECK(sensor.IsStale());

  auto& dev = AddChip(bus);
  SetFetch(dev, 0, 4096, 8191);
  sensor.Update();
  CHECK(sensor.IsConnected());
  sim::StepTiming(50_ms);
  dev.nack = true;
  sensor.Update();  // the fetch fails
  CHECK_FALSE(sensor.IsConnected());
  dev.nack = false;
  sensor.Update();  // a new request
  sim::StepTiming(50_ms);
  sensor.Update();
  CHECK(sensor.IsConnected());
  CHECK(sensor.GetHumidity() == Catch::Approx(4096 * 100.0 / 16382));
  sim::ResumeTiming();
}
