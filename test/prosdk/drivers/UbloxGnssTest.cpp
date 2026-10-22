// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/gnss/UbloxGnss.hpp"

#include "FakeI2CBus.hpp"

#include <algorithm>
#include <deque>
#include <span>
#include <string_view>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/simulation/SimHooks.hpp"

using namespace prosdk;

namespace {

using Bytes = std::vector<uint8_t>;

Bytes MonVer(std::string_view protver) {
  Bytes p(40 + 2 * 30, 0);
  std::string_view sw = "ROM SPG 5.10 (7b202e)";
  std::copy(sw.begin(), sw.end(), p.begin());
  std::copy(protver.begin(), protver.end(), p.begin() + 70);
  return ubx::Encode(ubx::kClassMon, ubx::kIdMonVer, p);
}

ubx::NavPvt SamplePvt() {
  ubx::NavPvt v;
  v.year = 2026;
  v.month = 10;
  v.day = 22;
  v.hour = 13;
  v.minute = 45;
  v.second = 59;
  v.valid = 0x07;
  v.nano = -250000;
  v.fixType = 3;
  v.flags = 0x01;
  v.numSv = 11;
  v.lon = 289784210;
  v.lat = 410082560;
  v.hMsl = 37456;
  v.hAcc = 1830;
  v.gSpeed = 361;
  v.headMot = 10955000;
  return v;
}

Bytes NavPvtFrame(const ubx::NavPvt& v) {
  auto p = ubx::EncodeNavPvt(v);
  return ubx::Encode(ubx::kClassNav, ubx::kIdNavPvt, p);
}

// A u-blox receiver on the DDC port (M8 Receiver Description section 11.5): a one-byte write sets
// the register address, 0xFD/0xFE give the number of bytes waiting, 0xFF streams them (0xFF when
// empty); longer writes are UBX frames. It answers MON-VER polls and acknowledges configuration
// messages (answerUbx false: it answers nothing); an M10 rejects UBX-CFG-PRT.
struct FakeUblox {
  FakeUblox(test::FakeI2CBus& bus, std::string_view protver, bool m10, int address = 0x42)
      : dev{bus.Add(address)}, protver{protver}, m10{m10} {
    dev.onWrite = [this](std::span<const uint8_t> data) {
      if (data.size() == 1) {
        reg = data[0];
        return;
      }
      for (uint8_t b : data) {
        if (parser.Push(b)) {
          auto m = parser.GetMessage();
          Bytes frame = ubx::Encode(m.messageClass, m.messageId, m.payload);
          frames.push_back(frame);
          Answer(m.messageClass, m.messageId, m.payload.size());
        }
      }
    };
    dev.onRead = [this](std::span<uint8_t> data) {
      for (uint8_t& b : data) {
        if (reg == 0xFD) {
          b = emptyCountFF && out.empty() ? 0xFF : static_cast<uint8_t>(out.size() >> 8);
          reg = 0xFE;
        } else if (reg == 0xFE) {
          b = emptyCountFF && out.empty() ? 0xFF : static_cast<uint8_t>(out.size() & 0xFF);
          reg = 0xFF;
        } else if (out.empty()) {
          b = 0xFF;
        } else {
          b = out.front();
          out.pop_front();
        }
      }
    };
  }
  FakeUblox(const FakeUblox&) = delete;
  FakeUblox& operator=(const FakeUblox&) = delete;

  void Queue(const Bytes& frame) { out.insert(out.end(), frame.begin(), frame.end()); }

  void Answer(uint8_t cls, uint8_t id, size_t length) {
    if (!answerUbx) {
      return;
    }
    if (cls == ubx::kClassMon && id == ubx::kIdMonVer && length == 0) {
      Queue(MonVer(protver));
    } else if (cls == ubx::kClassCfg) {
      bool ack = ackConfig && !(m10 && id == ubx::kIdCfgPrt);
      const uint8_t payload[] = {cls, id};
      Queue(ubx::Encode(ubx::kClassAck, ack ? ubx::kIdAckAck : ubx::kIdAckNak, payload));
    }
  }

  test::FakeI2CBus::Device& dev;
  std::string_view protver;
  bool m10;
  bool answerUbx = true;
  bool ackConfig = true;
  bool emptyCountFF = false;  // report 0xFFFF bytes waiting when empty
  int reg = 0xFF;
  ubx::Parser parser;
  std::deque<uint8_t> out;
  std::vector<Bytes> frames;  // frames received from the driver
};

void CheckSampleFix(const GnssFix& fix) {
  CHECK(fix.valid);
  CHECK(fix.fixType == GnssFixType::k3D);
  CHECK(fix.latitude.value() == Catch::Approx(41.008256));
  CHECK(fix.longitude.value() == Catch::Approx(28.978421));
  CHECK(fix.heightMsl.value() == Catch::Approx(37.456));
  CHECK(fix.groundSpeed.value() == Catch::Approx(0.361));
  CHECK(fix.heading.value() == Catch::Approx(109.55));
  CHECK(fix.numSatellites == 11);
  CHECK(fix.hAcc.value() == Catch::Approx(1.83));
  CHECK(fix.utc.year == 2026);
  CHECK(fix.utc.month == 10);
  CHECK(fix.utc.day == 22);
  CHECK(fix.utc.hour == 13);
  CHECK(fix.utc.minute == 45);
  CHECK(fix.utc.second == Catch::Approx(58.99975));
  CHECK(fix.utc.dateValid);
  CHECK(fix.utc.timeValid);
}

}  // namespace

TEST_CASE("UbloxGnss configures an M10 over I2C with CFG-VALSET", "[gnss][ublox]") {
  sim::PauseTiming();
  test::FakeI2CBus bus{0};
  FakeUblox rx{bus, "PROTVER=34.10", true};
  UbloxGnss gnss{I2C::Port::PORT_0, UbloxGnss::kDefaultAddress, 0_ms};
  CHECK_FALSE(gnss.IsConnected());
  CHECK_FALSE(gnss.IsConfigured());
  CHECK_FALSE(gnss.GetFix().valid);

  gnss.Update();  // probe
  REQUIRE(rx.frames.size() == 2);
  CHECK(rx.frames[0] == ubx::EncodeCfgPrtDdc(0x42));
  CHECK(rx.frames[1] == ubx::Encode(ubx::kClassMon, ubx::kIdMonVer));

  gnss.Update();  // reads the NAK of CFG-PRT and MON-VER, sends the configuration
  CHECK(gnss.IsConnected());
  REQUIRE(rx.frames.size() == 3);
  const ubx::ConfigItem items[] = {
      {ubx::key::kI2cOutprotNmea, 0},
      {ubx::key::kMsgoutNavPvtI2c, 1},
      {ubx::key::kRateMeas, 1000},
  };
  CHECK(rx.frames[2] == ubx::EncodeValset(ubx::kLayerRam, items));
  CHECK_FALSE(gnss.IsConfigured());

  gnss.Update();  // reads ACK-ACK
  CHECK(gnss.IsConfigured());
  CHECK(rx.frames.size() == 3);
  CHECK_FALSE(gnss.GetFix().valid);

  // NAV-PVT arrives
  rx.Queue(NavPvtFrame(SamplePvt()));
  gnss.Update();
  CheckSampleFix(gnss.GetFix());
  CHECK(gnss.GetChecksumErrors() == 0);

  // no NAV-PVT for longer than the timeout (3 navigation periods at 1 Hz): probe again
  sim::StepTiming(2500_ms);
  gnss.Update();
  CHECK(gnss.IsConfigured());
  CHECK(gnss.IsConnected());
  sim::StepTiming(600_ms);
  CHECK_FALSE(gnss.IsConnected());
  gnss.Update();
  CHECK_FALSE(gnss.IsConfigured());
  CHECK(gnss.GetFix().valid);  // the last fix is kept
  gnss.Update();
  REQUIRE(rx.frames.size() == 5);
  CHECK(rx.frames[3] == ubx::EncodeCfgPrtDdc(0x42));
  CHECK(rx.frames[4] == ubx::Encode(ubx::kClassMon, ubx::kIdMonVer));
  gnss.Update();
  gnss.Update();
  CHECK(gnss.IsConfigured());
  sim::ResumeTiming();
}

TEST_CASE("UbloxGnss configures a u-blox 8 with CFG-MSG and CFG-RATE", "[gnss][ublox]") {
  sim::PauseTiming();
  test::FakeI2CBus bus{1};
  FakeUblox rx{bus, "PROTVER=18.00", false};
  rx.emptyCountFF = true;
  UbloxGnss gnss{I2C::Port::PORT_1, UbloxGnss::kDefaultAddress, 0_ms};

  gnss.Update();
  gnss.Update();
  REQUIRE(rx.frames.size() == 4);
  CHECK(rx.frames[2] == ubx::EncodeCfgMsg(ubx::kClassNav, ubx::kIdNavPvt, 1));
  CHECK(rx.frames[3] == ubx::EncodeCfgRate(1000));
  gnss.Update();
  CHECK(gnss.IsConfigured());

  // a new rate is sent at once
  gnss.SetRate(units::hertz_t{5});
  CHECK_FALSE(gnss.IsConfigured());
  gnss.Update();
  REQUIRE(rx.frames.size() == 6);
  CHECK(rx.frames[4] == ubx::EncodeCfgMsg(ubx::kClassNav, ubx::kIdNavPvt, 1));
  CHECK(rx.frames[5] == ubx::EncodeCfgRate(200));
  gnss.Update();
  CHECK(gnss.IsConfigured());

  // a refused configuration is retried every second, three times, then the receiver is probed
  rx.ackConfig = false;
  gnss.SetRate(units::hertz_t{100});  // clamped to 50 ms
  gnss.Update();
  REQUIRE(rx.frames.size() == 8);
  CHECK(rx.frames[7] == ubx::EncodeCfgRate(50));
  gnss.Update();  // NAK: stays unconfigured, nothing resent yet
  CHECK_FALSE(gnss.IsConfigured());
  CHECK(rx.frames.size() == 8);
  sim::StepTiming(1100_ms);
  gnss.Update();
  CHECK(rx.frames.size() == 10);
  sim::StepTiming(1100_ms);
  gnss.Update();
  CHECK(rx.frames.size() == 12);
  sim::StepTiming(1100_ms);
  gnss.Update();  // third attempt failed: back to the probe
  gnss.Update();
  REQUIRE(rx.frames.size() == 14);
  CHECK(rx.frames[12] == ubx::EncodeCfgPrtDdc(0x42));
  CHECK(rx.frames[13] == ubx::Encode(ubx::kClassMon, ubx::kIdMonVer));
  CHECK_FALSE(gnss.IsConfigured());
  sim::ResumeTiming();
}

TEST_CASE("UbloxGnss without a receiver keeps probing", "[gnss][ublox]") {
  sim::PauseTiming();
  test::FakeI2CBus bus{0};  // nothing at 0x42: every transfer is NACKed
  UbloxGnss gnss{I2C::Port::PORT_0, UbloxGnss::kDefaultAddress, 0_ms};
  for (int i = 0; i < 6; ++i) {
    gnss.Update();
    sim::StepTiming(1100_ms);
  }
  CHECK_FALSE(gnss.IsConnected());
  CHECK_FALSE(gnss.IsConfigured());
  CHECK_FALSE(gnss.GetFix().valid);

  // a receiver that streams NMEA only and ignores UBX
  FakeUblox rx{bus, "PROTVER=18.00", false};
  rx.answerUbx = false;
  rx.Queue(Bytes{'$', 'G', 'N', 'T', 'X', 'T', '\r', '\n'});
  gnss.Update();
  sim::StepTiming(1100_ms);
  gnss.Update();
  CHECK_FALSE(gnss.IsConnected());
  CHECK_FALSE(gnss.IsConfigured());
  CHECK(rx.frames.size() == 4);  // two probes
  sim::ResumeTiming();
}

TEST_CASE("UbloxGnss parses a serial stream", "[gnss][ublox]") {
  sim::PauseTiming();
  UbloxGnss gnss{SerialPort::Port::ONBOARD, UbloxGnss::kDefaultBaudRate, 0_ms};
  CHECK(gnss.GetTelemetryType() == "GNSS");

  std::string_view nmea = "$GNGGA,,,,,,0,00,99.99,,,,,,*56\r\n";
  Bytes stream(nmea.begin(), nmea.end());
  Bytes pvt = NavPvtFrame(SamplePvt());
  stream.insert(stream.end(), pvt.begin(), pvt.end());
  gnss.Feed(std::span<const uint8_t>{stream}.first(60));
  CHECK_FALSE(gnss.GetFix().valid);
  gnss.Feed(std::span<const uint8_t>{stream}.subspan(60));
  CheckSampleFix(gnss.GetFix());
  CHECK(gnss.IsConnected());
  CHECK_FALSE(gnss.IsConfigured());  // configuration needs the receiver's answers

  // no gnssFixOK: the solution is reported but not valid
  ubx::NavPvt weak = SamplePvt();
  weak.flags = 0;
  weak.fixType = 2;
  gnss.Feed(NavPvtFrame(weak));
  CHECK_FALSE(gnss.GetFix().valid);
  CHECK(gnss.GetFix().fixType == GnssFixType::k2D);

  ubx::NavPvt timeOnly = SamplePvt();
  timeOnly.fixType = 5;
  gnss.Feed(NavPvtFrame(timeOnly));
  CHECK_FALSE(gnss.GetFix().valid);
  CHECK(gnss.GetFix().fixType == GnssFixType::kTimeOnly);

  Bytes bad = NavPvtFrame(SamplePvt());
  bad[30] ^= 0x10;
  gnss.Feed(bad);
  CHECK(gnss.GetChecksumErrors() == 1);
  CHECK(gnss.GetFix().fixType == GnssFixType::kTimeOnly);

  sim::StepTiming(2900_ms);
  CHECK(gnss.IsConnected());  // timeout: 3 navigation periods at 1 Hz
  sim::StepTiming(200_ms);
  CHECK_FALSE(gnss.IsConnected());
  sim::ResumeTiming();
}
