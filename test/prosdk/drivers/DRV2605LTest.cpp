// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/haptic/DRV2605L.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {
// The DRV2605L reset values that matter here (SLOS854D table 3).
test::FakeI2CBus::Device& AddChip(test::FakeI2CBus& bus) {
  auto& chip = bus.Add(DRV2605L::kAddress);
  chip.regs[0x00] = 0xE0;  // STATUS: DEVICE_ID 7
  chip.regs[0x01] = 0x40;  // MODE: standby
  chip.regs[0x03] = 0x01;  // LIBRARY A
  chip.regs[0x04] = 0x01;  // WAV_FRM_SEQ1
  chip.regs[0x1A] = 0x36;  // FEEDBACK: ERM
  return chip;
}
}  // namespace

TEST_CASE("DRV2605L configures the chip for the actuator", "[drv2605l]") {
  test::FakeI2CBus bus;
  auto& chip = AddChip(bus);
  {
    DRV2605L erm{I2C::Port::PORT_0};
    CHECK(erm.IsConnected());
    CHECK(erm.GetActuator() == DRV2605L::Actuator::kERM);
    CHECK(chip.regs[0x01] == 0x00);  // out of standby, internal trigger
    CHECK(chip.regs[0x1A] == 0x36);  // N_ERM_LRA = 0
    CHECK(chip.regs[0x03] == 1);     // TS2200 library A
    CHECK(chip.regs[0x02] == 0);
    for (int reg = 0x04; reg <= 0x0B; ++reg) {
      CHECK(chip.regs[reg] == 0);  // empty sequence
    }
  }
  CHECK(chip.regs[0x01] == 0x40);  // the destructor puts the chip in standby
  CHECK(chip.regs[0x0C] == 0);

  DRV2605L lra{I2C::Port::PORT_0, DRV2605L::Actuator::kLRA};
  CHECK(lra.IsConnected());
  CHECK(chip.regs[0x1A] == 0xB6);  // N_ERM_LRA = 1, other bits kept
  CHECK(chip.regs[0x03] == 6);     // LRA library
}

TEST_CASE("DRV2605L sequencer and GO", "[drv2605l]") {
  test::FakeI2CBus bus;
  auto& chip = AddChip(bus);
  DRV2605L drv{I2C::Port::PORT_0};
  drv.SetLibrary(7);
  CHECK(chip.regs[0x03] == 7);
  drv.SetWaveform(0, 47);
  drv.SetWaveform(1, 123);
  drv.SetWaveform(7, 0);
  CHECK(chip.regs[0x04] == 47);
  CHECK(chip.regs[0x05] == 123);
  CHECK(chip.regs[0x0B] == 0);

  chip.writes.clear();
  drv.Go();
  REQUIRE(chip.writes.size() == 1);
  CHECK(chip.writes[0] == std::vector<uint8_t>{0x0C, 0x01});
  CHECK(drv.IsPlaying());
  chip.regs[0x0C] = 0;  // the sequence ended: GO self-clears
  CHECK_FALSE(drv.IsPlaying());

  drv.Go();
  drv.Stop();
  CHECK(chip.regs[0x0C] == 0);

  CHECK_THROWS_AS(drv.SetLibrary(8), RuntimeError);
  CHECK_THROWS_AS(drv.SetLibrary(-1), RuntimeError);
  CHECK_THROWS_AS(drv.SetWaveform(8, 1), RuntimeError);
  CHECK_THROWS_AS(drv.SetWaveform(0, 124), RuntimeError);
}

TEST_CASE("DRV2605L real-time playback", "[drv2605l]") {
  test::FakeI2CBus bus;
  auto& chip = AddChip(bus);
  DRV2605L drv{I2C::Port::PORT_0};

  drv.SetRealtime(0.5);
  CHECK(chip.regs[0x02] == 0x40);  // round(0.5 * 127) = 64, signed format
  CHECK(chip.regs[0x01] == 0x05);  // RTP mode
  drv.SetRealtime(-1.0);
  CHECK(chip.regs[0x02] == 0x81);
  drv.SetRealtime(3.0);
  CHECK(chip.regs[0x02] == 0x7F);
  drv.SetRealtime(-3.0);
  CHECK(chip.regs[0x02] == 0x81);
  drv.SetRealtime(std::nan(""));
  CHECK(chip.regs[0x02] == 0x00);
  CHECK(chip.regs[0x01] == 0x05);

  drv.SetRealtime(0.25);
  drv.Go();  // leaves RTP mode before GO
  CHECK(chip.regs[0x01] == 0x00);
  CHECK(chip.regs[0x0C] == 0x01);

  drv.SetRealtime(0.25);
  CHECK(chip.regs[0x01] == 0x05);
  drv.Stop();
  CHECK(chip.regs[0x02] == 0x00);
  CHECK(chip.regs[0x01] == 0x00);
  CHECK(chip.regs[0x0C] == 0x00);
}

TEST_CASE("DRV2605L reports a missing or different chip", "[drv2605l]") {
  test::FakeI2CBus bus;
  {
    DRV2605L missing{I2C::Port::PORT_0};
    CHECK_FALSE(missing.IsConnected());
    CHECK_FALSE(missing.IsPlaying());
  }
  auto& chip = AddChip(bus);
  chip.regs[0x00] = 0x60;  // DEVICE_ID 3: DRV2605
  DRV2605L other{I2C::Port::PORT_0};
  CHECK_FALSE(other.IsConnected());
  chip.regs[0x00] = 0xE0;
  chip.nack = true;
  CHECK_FALSE(other.IsPlaying());
}

TEST_CASE("DRV2605L retries the configuration", "[drv2605l]") {
  test::FakeI2CBus bus;
  auto& chip = AddChip(bus);
  chip.nack = true;
  DRV2605L drv{I2C::Port::PORT_0, DRV2605L::Actuator::kLRA};
  CHECK_FALSE(drv.IsConnected());
  chip.nack = false;
  drv.SetWaveform(0, 10);  // configures first, then writes the slot
  CHECK(drv.IsConnected());
  CHECK(chip.regs[0x01] == 0x00);  // out of standby
  CHECK(chip.regs[0x1A] == 0xB6);  // LRA
  CHECK(chip.regs[0x03] == 6);
  CHECK(chip.regs[0x04] == 10);
  CHECK(chip.regs[0x05] == 0);
  chip.writes.clear();
  drv.SetWaveform(1, 11);  // configured: one write
  CHECK(chip.writes.size() == 1);
}
