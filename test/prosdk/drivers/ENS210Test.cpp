// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/environment/ENS210.hpp"

#include <cstdint>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/simulation/SimHooks.hpp"

using namespace prosdk;

namespace {
test::FakeI2CBus::Device& AddChip(test::FakeI2CBus& bus) {
  auto& dev = bus.Add(0x43);
  dev.regs[0x00] = 0x10;  // PART_ID 0x0210, little endian
  dev.regs[0x01] = 0x02;
  dev.regs[0x11] = 0x01;  // SYS_STAT: active
  // the data sheet example readout: T_VAL 0B49FD (CRC 05, valid), H_VAL F52E6C (CRC 7A, valid)
  const uint8_t values[] = {0xFD, 0x49, 0x0B, 0x6C, 0x2E, 0xF5};
  for (int i = 0; i < 6; ++i) {
    dev.regs[0x30 + i] = values[i];
  }
  return dev;
}
}  // namespace

TEST_CASE("ENS210 starts continuous mode and reads checked values", "[ens210]") {
  sim::PauseTiming();
  test::FakeI2CBus bus;
  auto& dev = AddChip(bus);
  ENS210 sensor{I2C::Port::PORT_0};
  sensor.Update();
  CHECK(sensor.IsConnected());
  REQUIRE(dev.writes.size() == 2);
  CHECK(dev.writes[0] == std::vector<uint8_t>{0x21, 0x03});  // SENS_RUN: T and RH continuous
  CHECK(dev.writes[1] == std::vector<uint8_t>{0x22, 0x03});  // SENS_START
  CHECK_FALSE(sensor.HasData());

  sensor.Update();  // SYS_STAT and PART_ID
  CHECK(dev.reads == 2);
  CHECK_FALSE(sensor.HasData());

  sensor.Update();  // one 6-byte burst from T_VAL
  CHECK(dev.reads == 3);
  CHECK(dev.writes.back() == std::vector<uint8_t>{0x30});
  CHECK(sensor.HasData());
  CHECK(sensor.GetTemperature().value() == Catch::Approx(0x49FD / 64.0 - 273.15));  // 22.80 C
  CHECK(sensor.GetHumidity() == Catch::Approx(0x2E6C / 512.0));                     // 23.21 %
  CHECK(sensor.GetCrcErrors() == 0);

  // a corrupted temperature byte fails the CRC: the old temperature stays, humidity updates
  dev.regs[0x30] = 0xFE;
  sensor.Update();
  CHECK(sensor.GetCrcErrors() == 1);
  CHECK(sensor.GetTemperature().value() == Catch::Approx(0x49FD / 64.0 - 273.15));
  sim::ResumeTiming();
}

TEST_CASE("ENS210 waits for the active state before reading PART_ID", "[ens210]") {
  sim::PauseTiming();
  test::FakeI2CBus bus;
  auto& dev = AddChip(bus);
  dev.regs[0x11] = 0x00;  // still booting
  ENS210 sensor{I2C::Port::PORT_0};
  sensor.Update();
  sensor.Update();
  CHECK(dev.reads == 1);  // SYS_STAT only
  dev.regs[0x11] = 0x01;
  sensor.Update();
  CHECK(dev.reads == 3);
  sensor.Update();
  CHECK(sensor.HasData());
  sim::ResumeTiming();
}

TEST_CASE("ENS210 retries a missing chip once per second", "[ens210]") {
  sim::PauseTiming();
  test::FakeI2CBus bus;
  ENS210 sensor{I2C::Port::PORT_0};
  sensor.Update();
  CHECK_FALSE(sensor.IsConnected());

  auto& dev = AddChip(bus);
  sim::StepTiming(500_ms);
  sensor.Update();  // within the retry period: no attempt
  CHECK(dev.writes.empty());
  sim::StepTiming(600_ms);
  sensor.Update();
  CHECK(sensor.IsConnected());
  CHECK(dev.writes.size() == 2);
  sensor.Update();
  sensor.Update();
  CHECK(sensor.HasData());

  dev.nack = true;
  sensor.Update();
  CHECK_FALSE(sensor.IsConnected());
  dev.nack = false;
  sim::StepTiming(1100_ms);
  sensor.Update();  // started again
  CHECK(sensor.IsConnected());
  CHECK(dev.writes.back() == std::vector<uint8_t>{0x22, 0x03});
  sim::ResumeTiming();
}
