// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/thermal/CaliPile.hpp"

#include <array>
#include <span>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/simulation/SimHooks.hpp"

using namespace prosdk;

namespace {
using Bytes = std::vector<uint8_t>;

// EEPROM cells 0-31 (registers 32-63) with the typical values of the TPiS 1S 1385 specification,
// Table 37: PROTOCOL 3, LOOKUP# 1, PTAT25 13500, M 17200 (172 counts/K), U0 31732 (64500),
// UOUT1 35500 (71000), TOBJ1 40 C, slave address 0x8C.
std::array<uint8_t, 32> Eeprom() {
  std::array<uint8_t, 32> ee{};
  ee[0] = 3;
  ee[9] = 1;
  ee[10] = 13500 >> 8;
  ee[11] = 13500 & 0xFF;
  ee[12] = 17200 >> 8;
  ee[13] = 17200 & 0xFF;
  ee[14] = 31732 >> 8;
  ee[15] = 31732 & 0xFF;
  ee[16] = 35500 >> 8;
  ee[17] = 35500 & 0xFF;
  ee[18] = 40;
  ee[31] = 0x8C;
  unsigned sum = ee[0];  // all cells except the checksum cells 1 and 2
  for (int i = 3; i < 32; ++i) {
    sum += ee[i];
  }
  ee[1] = static_cast<uint8_t>(sum >> 8);
  ee[2] = static_cast<uint8_t>(sum);
  return ee;
}

// TPobject in registers 1, 2, 3[7]; TPambient in 3[6:0], 4
void SetReading(test::FakeI2CBus::Device& dev, uint32_t tpObject, uint32_t tpAmbient) {
  dev.regs[1] = static_cast<uint8_t>(tpObject >> 9);
  dev.regs[2] = static_cast<uint8_t>(tpObject >> 1);
  dev.regs[3] = static_cast<uint8_t>(((tpObject & 1) << 7) | ((tpAmbient >> 8) & 0x7F));
  dev.regs[4] = static_cast<uint8_t>(tpAmbient);
}

// A CaliPile at 0x0C that answers only after the general call reload, as after power-up
// (specification 5.3 and 5.5).
struct Fixture {
  test::FakeI2CBus bus;
  test::FakeI2CBus::Device& chip = bus.Add(CaliPile::kDefaultAddress);
  test::FakeI2CBus::Device& generalCall = bus.Add(0x00);

  Fixture() {
    chip.nack = true;
    auto ee = Eeprom();
    for (int i = 0; i < 32; ++i) {
      chip.regs[32 + i] = ee[i];
    }
    generalCall.onWrite = [this](std::span<const uint8_t> data) {
      if (data.size() == 1 && data[0] == 0x04) {
        chip.nack = false;
      }
    };
    // Tamb 30 C: 13500 + 5 K * 172; Tobj 40 C: k * (f(313.15) - f(303.15)) + U0 with
    // f(x) = x^3.8 and k = (71000 - 64500) / (f(313.15) - f(298.15)), rounded
    SetReading(chip, 68932, 14360);
  }
};
}  // namespace

TEST_CASE("CaliPile reloads by general call, reads the EEPROM and converts", "[calipile]") {
  using Catch::Approx;
  Fixture f;
  f.chip.regs[19] = 0x08;  // chip status: presence flag

  CaliPile sensor{I2C::Port::PORT_0, CaliPile::Variant::k1S1385};
  CHECK(f.generalCall.writes.empty());  // no bus access before the first Update
  CHECK(f.chip.writes.empty());

  sensor.Update();
  REQUIRE(sensor.IsConnected());
  CHECK(f.generalCall.writes == std::vector<Bytes>{{0x04}});
  REQUIRE(f.chip.writes.size() == 5);
  CHECK(f.chip.writes[0] == Bytes{31, 0x80});  // ECR: EEPROM read access
  CHECK(f.chip.writes[1] == Bytes{32});        // EEPROM read pointer
  CHECK(f.chip.writes[2] == Bytes{31, 0x00});  // ECR back to idle
  CHECK(f.chip.writes[3] ==
        Bytes{20, 0x8B, 0x0A, 50, 10, 10, 0x00, 0x16, 0x00, 0xFF, 0xFF});  // registers 20-29
  CHECK(f.chip.writes[4] == Bytes{1});  // measurement read pointer
  CHECK(f.chip.regs[31] == 0x00);

  CHECK(sensor.GetAmbientTemperature().value() == Approx(30.0).margin(1e-9));
  CHECK(sensor.GetObjectTemperature().value() == Approx(40.0).margin(0.01));
  CHECK(sensor.IsPresence());
  CHECK_FALSE(sensor.IsMotion());
  CHECK_FALSE(sensor.IsOverTemperature());  // no threshold set

  sensor.SetOverTemperatureThreshold(35_degC);
  f.chip.regs[19] = 0x14;  // over-temperature and motion flags
  sensor.Update();
  // TPobject for 35 C at Tamb 30 C is 66665.7 counts; the register holds half
  CHECK(f.chip.writes.back() == Bytes{28, 0x82, 0x35});
  CHECK(f.chip.regs[28] == 0x82);
  CHECK(f.chip.regs[29] == 0x35);
  CHECK(sensor.IsOverTemperature());
  CHECK(sensor.IsMotion());
  CHECK_FALSE(sensor.IsPresence());

  std::size_t writes = f.chip.writes.size();
  sensor.Update();  // same ambient temperature: the threshold is not written again
  CHECK(f.chip.writes.size() == writes + 1);  // only the read pointer

  sensor.SetEmissivity(0.9);  // k scaled by 0.9 (specification 8.5)
  sensor.Update();
  CHECK(sensor.GetObjectTemperature().value() == Approx(41.0573).margin(0.01));
  CHECK(f.generalCall.writes.size() == 1);  // the reload happens once
}

TEST_CASE("CaliPile rejects a bad EEPROM checksum and retries after a second", "[calipile]") {
  Fixture f;
  f.chip.regs[33] ^= 0x01;
  sim::PauseTiming();

  CaliPile sensor{I2C::Port::PORT_0, CaliPile::Variant::k1S1385};
  sensor.Update();
  CHECK_FALSE(sensor.IsConnected());
  CHECK(f.chip.writes.size() == 3);  // ECR, EEPROM pointer, ECR idle: no configuration
  CHECK(f.chip.regs[31] == 0x00);
  CHECK(sensor.GetObjectTemperature().value() == 0.0);

  sensor.Update();  // within a second: no new attempt
  CHECK(f.generalCall.writes.size() == 1);

  f.chip.regs[33] ^= 0x01;
  sim::StepTiming(1.1_s);
  sensor.Update();
  CHECK(f.generalCall.writes.size() == 2);
  CHECK(sensor.IsConnected());
  CHECK(sensor.GetAmbientTemperature().value() == Catch::Approx(30.0).margin(1e-9));
  sim::ResumeTiming();
}

TEST_CASE("CaliPile without an answer to the general call stays disconnected", "[calipile]") {
  test::FakeI2CBus bus;  // no device at all
  CaliPile sensor{I2C::Port::PORT_0, CaliPile::Variant::k1T1386, 0x0E};
  sensor.Update();
  CHECK_FALSE(sensor.IsConnected());
  CHECK(sensor.GetAmbientTemperature().value() == 0.0);
}

TEST_CASE("CaliPile sets the chip up again after a power loss", "[calipile]") {
  Fixture f;
  sim::PauseTiming();

  CaliPile sensor{I2C::Port::PORT_0, CaliPile::Variant::k1S1385};
  sensor.Update();
  REQUIRE(sensor.IsConnected());

  f.chip.nack = true;  // power cycled: the chip answers only the general call again
  sensor.Update();
  CHECK_FALSE(sensor.IsConnected());
  sensor.Update();  // within a second of the last setup: no new attempt
  CHECK(f.generalCall.writes.size() == 1);

  sim::StepTiming(1.1_s);
  sensor.Update();
  CHECK(f.generalCall.writes.size() == 2);
  CHECK(sensor.IsConnected());
  CHECK(f.chip.writes.back() == Bytes{1});  // set up again and read in the same Update
  CHECK(sensor.GetAmbientTemperature().value() == Catch::Approx(30.0).margin(1e-9));
  sim::ResumeTiming();
}
