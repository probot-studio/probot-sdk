// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/dac/GP8403.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

using Bytes = std::vector<uint8_t>;

TEST_CASE("GP8403 sets the range and 0 V at construction", "[gp8403]") {
  test::FakeI2CBus bus;
  auto& dac = bus.Add(0x58);
  {
    GP8403 gp{I2C::Port::PORT_0};
    CHECK(gp.IsConnected());
    CHECK(gp.GetAddress() == 0x58);
    CHECK(gp.GetRange() == GP8403::Range::k0to5V);
    REQUIRE(dac.writes.size() == 2);
    CHECK(dac.writes[0] == Bytes{0x02, 0, 0, 0, 0});  // both outputs 0 V first
    CHECK(dac.writes[1] == Bytes{0x01, 0x00});        // 0-5 V
    gp.SetVoltage(1, units::volt_t{1.0});
    dac.writes.clear();
  }
  REQUIRE(dac.writes.size() == 1);
  CHECK(dac.writes[0] == Bytes{0x02, 0, 0, 0, 0});  // the destructor sets 0 V

  auto& dac2 = bus.Add(0x5B);
  GP8403 gp2{I2C::Port::PORT_0, 0x5B, GP8403::Range::k0to10V};
  REQUIRE(dac2.writes.size() == 2);
  CHECK(dac2.writes[1] == Bytes{0x01, 0x11});  // 0-10 V

  GP8403 missing{I2C::Port::PORT_0, 0x5C};
  CHECK_FALSE(missing.IsConnected());
}

TEST_CASE("GP8403 codes for 2.5 V at both ranges", "[gp8403]") {
  test::FakeI2CBus bus;
  auto& dac = bus.Add(0x58);
  GP8403 gp{I2C::Port::PORT_0};
  dac.writes.clear();

  // 0-5 V: round(2.5 / 5 * 4095) = 2048 -> left-justified 0x8000, low byte first
  gp.SetVoltage(0, units::volt_t{2.5});
  REQUIRE(dac.writes.size() == 1);
  CHECK(dac.writes[0] == Bytes{0x02, 0x00, 0x80});
  CHECK(gp.GetVoltage(0).value() == Catch::Approx(2048 * 5.0 / 4095));
  gp.SetVoltage(0, units::volt_t{2.5});
  CHECK(dac.writes.size() == 1);  // unchanged code: no transfer

  // 0-10 V: round(2.5 / 10 * 4095) = 1024 -> 0x4000
  gp.SetRange(GP8403::Range::k0to10V);
  REQUIRE(dac.writes.size() == 4);
  CHECK(dac.writes[1] == Bytes{0x02, 0, 0, 0, 0});      // outputs to 0 V
  CHECK(dac.writes[2] == Bytes{0x01, 0x11});            // range
  CHECK(dac.writes[3] == Bytes{0x02, 0x00, 0x40, 0, 0});  // stored 2.5 V again
  CHECK(gp.GetRange() == GP8403::Range::k0to10V);
  gp.SetVoltage(1, units::volt_t{2.5});
  REQUIRE(dac.writes.size() == 5);
  CHECK(dac.writes[4] == Bytes{0x04, 0x00, 0x40});
  CHECK(gp.GetVoltage(1).value() == Catch::Approx(1024 * 10.0 / 4095));

  // clamping
  gp.SetBoth(units::volt_t{20.0});
  CHECK(dac.writes.back() == Bytes{0x02, 0xF0, 0xFF, 0xF0, 0xFF});  // 4095 << 4
  CHECK(gp.GetVoltage(0).value() == Catch::Approx(10.0));
  gp.SetVoltage(0, units::volt_t{-1.0});
  CHECK(dac.writes.back() == Bytes{0x02, 0x00, 0x00});
  gp.SetVoltage(1, units::volt_t{std::nan("")});
  CHECK(dac.writes.back() == Bytes{0x04, 0x00, 0x00});

  // a 10 V output shrinks to the 5 V full scale
  gp.SetVoltage(0, units::volt_t{8.0});
  gp.SetRange(GP8403::Range::k0to5V);
  CHECK(dac.writes.back() == Bytes{0x02, 0xF0, 0xFF, 0x00, 0x00});

  CHECK_THROWS_AS(gp.SetVoltage(2, units::volt_t{1.0}), RuntimeError);
  CHECK_THROWS_AS(gp.GetVoltage(-1), RuntimeError);
}

TEST_CASE("GP8403 retries a failed write", "[gp8403]") {
  test::FakeI2CBus bus;
  auto& dac = bus.Add(0x58);
  GP8403 gp{I2C::Port::PORT_0};
  dac.nack = true;
  gp.SetVoltage(0, units::volt_t{1.0});
  CHECK_FALSE(gp.IsConnected());
  dac.nack = false;
  dac.writes.clear();
  gp.SetVoltage(0, units::volt_t{1.0});  // same code, but not written yet
  REQUIRE(dac.writes.size() == 1);
  CHECK(gp.IsConnected());
}

TEST_CASE("GP8403 writes the range before any code after a failed construction", "[gp8403]") {
  test::FakeI2CBus bus;
  auto& dac = bus.Add(0x58);
  dac.nack = true;
  GP8403 gp{I2C::Port::PORT_0, 0x58, GP8403::Range::k0to10V};
  CHECK_FALSE(gp.IsConnected());
  dac.nack = false;
  dac.writes.clear();
  gp.SetVoltage(0, units::volt_t{2.5});
  REQUIRE(dac.writes.size() == 3);
  CHECK(dac.writes[0] == Bytes{0x02, 0, 0, 0, 0});        // outputs to 0 V first
  CHECK(dac.writes[1] == Bytes{0x01, 0x11});              // 0-10 V
  CHECK(dac.writes[2] == Bytes{0x02, 0x00, 0x40, 0, 0});  // then the stored codes
  CHECK(gp.IsConnected());
  gp.SetVoltage(1, units::volt_t{2.5});
  REQUIRE(dac.writes.size() == 4);
  CHECK(dac.writes[3] == Bytes{0x04, 0x00, 0x40});  // range known: one channel only
}
