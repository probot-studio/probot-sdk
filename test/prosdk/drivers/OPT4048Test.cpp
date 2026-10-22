// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/light/OPT4048.hpp"

#include <array>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"

using namespace prosdk;

namespace {
// Channels 0-3 as (exponent, mantissa, counter): (2, 0x12345, 5), (3, 0x0ABCD, 6),
// (1, 0x54321, 7), (0, 0x00100, 8). The CRC nibbles follow datasheet Figure 8-12 with the counter
// in place of CRC_CHx; computed by hand for this test (unverified against a chip).
constexpr std::array<uint16_t, 8> kOutputs = {0x2123, 0x4552, 0x30AB, 0xCD60,
                                              0x1543, 0x2173, 0x0001, 0x0086};
// ADC codes (equation 3): 298260, 351848, 689730, 256; times the equation 16 matrix
constexpr double kX = 148.4455103641088;
constexpr double kY = 52.639027084087;
constexpr double kZ = 462.907153119932;
constexpr double kLux = 756.4732;  // equation 9: 351848 * 2.15e-3

test::FakeI2CBus::Device& AddOpt4048(test::FakeI2CBus& bus, uint16_t deviceId) {
  auto& dev = bus.Add(OPT4048::kDefaultAddress, 8, 16);  // 16-bit registers, MSB first
  dev.regs[0x11] = deviceId;
  dev.regs[0x0A] = 0x3208;  // reset value: power-down
  for (std::size_t i = 0; i < kOutputs.size(); ++i) {
    dev.regs[i] = kOutputs[i];
  }
  return dev;
}
}  // namespace

TEST_CASE("OPT4048 decodes a channel and its CRC", "[opt4048]") {
  OPT4048::Channel ch = OPT4048::DecodeChannel(0x2123, 0x4552);
  CHECK(ch.exponent == 2);
  CHECK(ch.mantissa == 0x12345);
  CHECK(ch.counter == 5);
  CHECK(ch.crc == 2);
  CHECK(OPT4048::ChannelCrc(ch) == 2);

  ch.mantissa ^= 0x10;  // one flipped bit changes the CRC
  CHECK(OPT4048::ChannelCrc(ch) != 2);

  OPT4048::Channel zero = OPT4048::DecodeChannel(0x0000, 0x0000);
  CHECK(OPT4048::ChannelCrc(zero) == 0);
}

TEST_CASE("OPT4048 reads XYZ, lux and CIE xy", "[opt4048]") {
  using Catch::Approx;
  test::FakeI2CBus bus;
  auto& dev = AddOpt4048(bus, 0x0821);

  OPT4048 sensor{I2C::Port::PORT_0};
  CHECK(sensor.IsConnected());
  CHECK(dev.regs[0x0A] == 0x3238);  // automatic range, 100 ms per channel, continuous

  sensor.Update();
  CHECK(sensor.IsConnected());
  CHECK(sensor.GetCrcErrors() == 0);
  OPT4048::XYZ xyz = sensor.GetXYZ();
  CHECK(xyz.x == Approx(kX));
  CHECK(xyz.y == Approx(kY));
  CHECK(xyz.z == Approx(kZ));
  CHECK(sensor.GetLux().value() == Approx(kLux));
  OPT4048::Chromaticity cie = sensor.GetCIExy();
  CHECK(cie.x == Approx(kX / (kX + kY + kZ)));
  CHECK(cie.y == Approx(kY / (kX + kY + kZ)));

  dev.regs[0x03] ^= 0x0100;  // a bit error in channel 1's mantissa: the sample is dropped
  sensor.Update();
  CHECK(sensor.GetCrcErrors() == 1);
  CHECK(sensor.GetLux().value() == Approx(kLux));

  dev.regs[0x03] ^= 0x0100;
  dev.regs[0x0A] = 0x3208;  // power loss: the chip is back in power-down
  sensor.Update();
  CHECK(dev.regs[0x0A] == 0x3238);
  CHECK(sensor.GetCrcErrors() == 1);
}

TEST_CASE("OPT4048 accepts both documented device IDs only", "[opt4048]") {
  {
    test::FakeI2CBus bus;
    AddOpt4048(bus, 0x0820);
    OPT4048 sensor{I2C::Port::PORT_0};
    CHECK(sensor.IsConnected());
  }
  {
    test::FakeI2CBus bus;
    auto& dev = AddOpt4048(bus, 0x3001);
    OPT4048 sensor{I2C::Port::PORT_0};
    CHECK_FALSE(sensor.IsConnected());
    CHECK(dev.regs[0x0A] == 0x3208);  // not configured
  }
}

TEST_CASE("OPT4048 shows darkness as zero chromaticity", "[opt4048]") {
  test::FakeI2CBus bus;
  auto& dev = AddOpt4048(bus, 0x0821);
  for (int i = 0; i < 8; ++i) {
    dev.regs[i] = 0;  // all zero: CRC 0 is valid
  }
  OPT4048 sensor{I2C::Port::PORT_0};
  sensor.Update();
  CHECK(sensor.GetCrcErrors() == 0);
  CHECK(sensor.GetLux().value() == 0.0);
  CHECK(sensor.GetCIExy().x == 0.0);
  CHECK(sensor.GetCIExy().y == 0.0);
}
