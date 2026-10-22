// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/adc/NAU7802.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/simulation/SimHooks.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {

using RegWrite = std::pair<int, int>;  // register, value

// NAU7802 register behaviour (data sheet Rev 1.7, 11.1 to 11.15): RR resets every register, PUR
// shows a few PU_CTRL reads after PUD, CR shows a ready conversion while CS is set, reading 0x12
// latches the result and clears CR (assumed: the data sheet does not say what clears CR), CALS
// clears itself a few CTRL2 reads after it was set.
struct FakeNau7802 {
  explicit FakeNau7802(test::FakeI2CBus& bus) : dev{bus.Add(0x2A)} {
    Reset();
    dev.onWrite = [this](std::span<const uint8_t> b) {
      if (b.empty()) {
        return;
      }
      pointer = b[0];
      for (std::size_t i = 1; i < b.size(); ++i) {
        Store(pointer++, b[i]);
      }
    };
    dev.onRead = [this](std::span<uint8_t> b) {
      for (auto& byte : b) {
        byte = Load(pointer++);
      }
    };
  }

  void Reset() {
    regs.fill(0);
    regs[0x07] = 0x80;  // gain calibration 1.0
    regs[0x0E] = 0x80;
    regs[0x1F] = 0x0F;  // revision ID 1111
    purLeft = 0;
  }

  void Store(int reg, uint8_t v) {
    regWrites.emplace_back(reg, v);
    if (reg == 0x00) {
      if (v & 0x01) {
        Reset();
        regs[0] = 0x01;
        return;
      }
      if ((v & 0x02) && !(regs[0] & 0x02)) {
        purLeft = purAfterReads;
      }
      regs[0] = v & 0xD7;  // CR and PUR are read only
    } else if (reg == 0x02) {
      regs[2] = v & 0xF7;  // CAL_ERR is read only
      if (v & 0x04) {
        calLeft = calReads;
        ++calibrations;
      }
    } else if (reg < 0x12 || reg > 0x14) {
      regs[reg & 0x1F] = v;
    }
  }

  uint8_t Load(int reg) {
    reg &= 0x1F;
    if (reg == 0x00) {
      ++puReads;
      uint8_t v = regs[0] & 0xD7;
      bool pur = false;
      if (regs[0] & 0x02) {
        if (purLeft > 0) {
          --purLeft;
        } else {
          pur = true;
        }
      }
      if (pur) {
        v |= 0x08;
      }
      if (pur && ready && (regs[0] & 0x10)) {
        v |= 0x20;
      }
      return v;
    }
    if (reg == 0x02 && calLeft > 0 && --calLeft == 0) {
      regs[2] = static_cast<uint8_t>((regs[2] & ~0x04) | (calError ? 0x08 : 0x00));
    }
    if (reg == 0x12) {
      latched = adc;
      ready = false;
      ++adcReads;
      return static_cast<uint8_t>(latched >> 16);
    }
    if (reg == 0x13) {
      return static_cast<uint8_t>(latched >> 8);
    }
    if (reg == 0x14) {
      return static_cast<uint8_t>(latched);
    }
    return regs[reg];
  }

  void Convert(uint32_t value) {
    adc = value & 0xFFFFFF;
    ready = true;
  }

  test::FakeI2CBus::Device& dev;
  std::array<uint8_t, 32> regs{};
  int pointer = 0;
  int purAfterReads = 2;  // PU_CTRL reads without PUR after PUD
  int purLeft = 0;
  int calReads = 3;       // CTRL2 reads until CALS clears
  int calLeft = 0;
  bool calError = false;
  int calibrations = 0;
  bool ready = false;
  uint32_t adc = 0;
  uint32_t latched = 0;
  int puReads = 0;
  int adcReads = 0;
  std::vector<RegWrite> regWrites;  // data writes only (not the register-pointer writes)
};

const std::vector<RegWrite> kSetup{
    {0x00, 0x01},  // RR
    {0x00, 0x02},  // PUD
    {0x01, 0x2F},  // CTRL1: LDO 3.0 V, gain 128
    {0x00, 0x86},  // AVDDS | PUA | PUD
    {0x15, 0x30},  // chopper clock off
    {0x02, 0x30},  // CTRL2: 80 SPS, channel 1
    {0x00, 0x96},  // + CS
};

// Drops the conversions after a change; returns how many were dropped before one came through.
int DroppedUntilValue(FakeNau7802& chip, NAU7802& adc, uint32_t value) {
  for (int i = 0; i < 20; ++i) {
    chip.Convert(value);
    if (adc.Update()) {
      return i;
    }
  }
  return -1;
}

}  // namespace

TEST_CASE("NAU7802 power-up sequence", "[nau7802]") {
  test::FakeI2CBus bus;
  FakeNau7802 chip{bus};
  chip.regs[0x1C] = 0x80;  // left set by another library: the reset clears it
  NAU7802 adc{I2C::Port::PORT_0};
  CHECK(adc.IsConnected());
  CHECK(chip.regWrites == kSetup);
  CHECK(chip.puReads >= 3);  // polled until PUR
  CHECK(chip.regs[0x1C] == 0x00);  // PGA_CAP_EN off: channel 2 stays usable
  CHECK(chip.calibrations == 0);
}

TEST_CASE("NAU7802 reads signed 24-bit conversions", "[nau7802]") {
  test::FakeI2CBus bus;
  FakeNau7802 chip{bus};
  NAU7802 adc{I2C::Port::PORT_0};

  CHECK_FALSE(adc.IsReady());
  CHECK_FALSE(adc.Update().has_value());
  CHECK(chip.adcReads == 0);  // no result read without CR

  chip.Convert(0x7FFFFF);
  CHECK(adc.IsReady());
  CHECK(adc.Update() == std::optional<int32_t>{8388607});
  CHECK(chip.adcReads == 1);
  CHECK_FALSE(adc.Update().has_value());  // CR cleared by the read
  CHECK(adc.GetRaw() == 8388607);

  chip.Convert(0x800000);
  CHECK(adc.Update() == std::optional<int32_t>{-8388608});
  chip.Convert(0xFFFFFF);
  CHECK(adc.Update() == std::optional<int32_t>{-1});
  chip.Convert(0x000001);
  CHECK(adc.Update() == std::optional<int32_t>{1});
  CHECK(adc.GetRaw() == 1);
  CHECK(adc.IsConnected());
}

TEST_CASE("NAU7802 gain, rate and channel", "[nau7802]") {
  test::FakeI2CBus bus;
  FakeNau7802 chip{bus};
  NAU7802 adc{I2C::Port::PORT_0};

  adc.SetGain(64);
  CHECK(chip.regWrites.back() == RegWrite{0x01, 0x2E});
  CHECK(DroppedUntilValue(chip, adc, 100) == 6);
  adc.SetGain(64);  // unchanged: nothing dropped
  CHECK(DroppedUntilValue(chip, adc, 101) == 0);
  adc.SetGain(1);
  CHECK(chip.regWrites.back() == RegWrite{0x01, 0x28});

  adc.SetSampleRate(320);
  CHECK(chip.regWrites.back() == RegWrite{0x02, 0x70});
  adc.SetChannel(2);
  CHECK(chip.regWrites.back() == RegWrite{0x02, 0xF0});
  CHECK(DroppedUntilValue(chip, adc, 102) == 6);
  adc.SetSampleRate(10);
  CHECK(chip.regWrites.back() == RegWrite{0x02, 0x80});
  adc.SetChannel(1);
  CHECK(chip.regWrites.back() == RegWrite{0x02, 0x00});

  auto writes = chip.regWrites.size();
  CHECK_THROWS_AS(adc.SetGain(3), RuntimeError);
  CHECK_THROWS_AS(adc.SetGain(256), RuntimeError);
  CHECK_THROWS_AS(adc.SetGain(0), RuntimeError);
  CHECK_THROWS_AS(adc.SetSampleRate(160), RuntimeError);
  CHECK_THROWS_AS(adc.SetChannel(0), RuntimeError);
  CHECK_THROWS_AS(adc.SetChannel(3), RuntimeError);
  CHECK(chip.regWrites.size() == writes);
}

TEST_CASE("NAU7802 internal calibration", "[nau7802]") {
  test::FakeI2CBus bus;
  FakeNau7802 chip{bus};
  NAU7802 adc{I2C::Port::PORT_0};

  CHECK(adc.CalibrateInternal());
  CHECK(chip.calibrations == 1);
  CHECK(chip.regWrites.back() == RegWrite{0x02, 0x34});  // 80 SPS | CALS, CALMOD 00
  CHECK((chip.regs[2] & 0x04) == 0);
  CHECK(DroppedUntilValue(chip, adc, 5) == 6);

  adc.SetChannel(2);
  chip.calError = true;
  CHECK_FALSE(adc.CalibrateInternal());
  CHECK(chip.regWrites.back() == RegWrite{0x02, 0xB4});  // channel 2
  chip.calError = false;
  CHECK(adc.CalibrateInternal());

  chip.dev.nack = true;
  CHECK_FALSE(adc.CalibrateInternal());
  CHECK_FALSE(adc.IsConnected());
}

TEST_CASE("NAU7802 missing chip, NACK and set up again", "[nau7802]") {
  sim::PauseTiming();
  test::FakeI2CBus bus;
  NAU7802 adc{I2C::Port::PORT_0};
  CHECK_FALSE(adc.IsConnected());
  CHECK_FALSE(adc.Update().has_value());
  CHECK_FALSE(adc.IsConnected());

  // the board is plugged in later: powered on, not set up (PU_CTRL 0)
  FakeNau7802 chip{bus};
  chip.Convert(42);
  CHECK_FALSE(adc.Update().has_value());
  CHECK(chip.regWrites.empty());  // less than 1 s since the last attempt
  sim::StepTiming(1.1_s);
  CHECK_FALSE(adc.Update().has_value());
  CHECK(chip.regWrites == kSetup);
  CHECK(adc.IsConnected());
  chip.Convert(42);
  CHECK(adc.Update() == std::optional<int32_t>{42});

  chip.dev.nack = true;
  chip.Convert(43);
  CHECK_FALSE(adc.Update().has_value());
  CHECK_FALSE(adc.IsConnected());
  CHECK(adc.GetRaw() == 42);
  chip.dev.nack = false;
  CHECK(adc.Update() == std::optional<int32_t>{43});
  CHECK(adc.IsConnected());
  sim::ResumeTiming();
}
