// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/gnss/Ubx.hpp"

#include <algorithm>
#include <array>
#include <iterator>
#include <span>
#include <string_view>
#include <vector>

#include <catch2/catch_test_macros.hpp>

using namespace prosdk;

namespace {

using Bytes = std::vector<uint8_t>;

// Pushes every byte; returns (class, id, payload) of the frames the parser accepted.
struct Frame {
  uint8_t messageClass;
  uint8_t messageId;
  Bytes payload;
};

std::vector<Frame> PushAll(ubx::Parser& parser, std::span<const uint8_t> bytes) {
  std::vector<Frame> out;
  for (uint8_t b : bytes) {
    if (parser.Push(b)) {
      auto m = parser.GetMessage();
      out.push_back({m.messageClass, m.messageId, Bytes(m.payload.begin(), m.payload.end())});
    }
  }
  return out;
}

ubx::NavPvt SamplePvt() {
  ubx::NavPvt v;
  v.iTow = 123456789;
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
  v.lon = 289784210;   // 28.978421 E
  v.lat = 410082560;   // 41.008256 N
  v.height = 75123;
  v.hMsl = 37456;
  v.hAcc = 1830;
  v.vAcc = 2950;
  v.velN = -120;
  v.velE = 340;
  v.velD = -5;
  v.gSpeed = 361;
  v.headMot = 10955000;  // 109.55 deg
  v.pDop = 145;
  return v;
}

Bytes MonVer(std::string_view extension) {
  Bytes p(40 + 2 * 30, 0);
  std::string_view sw = "ROM SPG 5.10 (7b202e)";
  std::copy(sw.begin(), sw.end(), p.begin());
  std::string_view hw = "000A0000";
  std::copy(hw.begin(), hw.end(), p.begin() + 30);
  std::string_view first = "FWVER=SPG 5.10";
  std::copy(first.begin(), first.end(), p.begin() + 40);
  std::copy(extension.begin(), extension.end(), p.begin() + 70);
  return p;
}

}  // namespace

TEST_CASE("UBX frames match known-good byte sequences", "[gnss][ubx]") {
  // UBX-MON-VER poll, UBX-CFG-RATE 1 Hz and UBX-CFG-MSG NAV-PVT at rate 1, as published for
  // u-blox receivers; checksums by the 8-bit Fletcher algorithm (M10 section 3.4).
  CHECK(ubx::Encode(ubx::kClassMon, ubx::kIdMonVer) ==
        Bytes{0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34});
  CHECK(ubx::EncodeCfgRate(1000) ==
        Bytes{0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39});
  CHECK(ubx::EncodeCfgMsg(ubx::kClassNav, ubx::kIdNavPvt, 1) ==
        Bytes{0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51});
  const uint8_t ck[] = {0x0A, 0x04, 0x00, 0x00};
  CHECK(ubx::Checksum(ck) == std::array<uint8_t, 2>{0x0E, 0x34});
}

TEST_CASE("UBX configuration key value sizes", "[gnss][ubx]") {
  CHECK(ubx::KeyValueSize(ubx::key::kI2cOutprotNmea) == 1);     // L
  CHECK(ubx::KeyValueSize(ubx::key::kUart1OutprotNmea) == 1);   // L
  CHECK(ubx::KeyValueSize(ubx::key::kMsgoutNavPvtI2c) == 1);    // U1
  CHECK(ubx::KeyValueSize(ubx::key::kMsgoutNavPvtUart1) == 1);  // U1
  CHECK(ubx::KeyValueSize(ubx::key::kRateMeas) == 2);           // U2
  CHECK(ubx::KeyValueSize(0x40520001) == 4);                    // CFG-UART1-BAUDRATE, U4
  CHECK(ubx::KeyValueSize(0x50000000) == 8);
  CHECK(ubx::KeyValueSize(0x00000000) == 0);
  CHECK(ubx::KeyValueSize(0x60000000) == 0);
}

TEST_CASE("UBX-CFG-VALSET byte layout", "[gnss][ubx]") {
  // M10 section 3.10.5: version U1 = 0, layers X1, reserved0 U1[2], then cfgData: each key U4 LE
  // followed by its value in KeyValueSize bytes, no padding.
  const ubx::ConfigItem items[] = {
      {ubx::key::kRateMeas, 100},
      {ubx::key::kUart1OutprotNmea, 0},
      {ubx::key::kMsgoutNavPvtUart1, 1},
  };
  CHECK(ubx::EncodeValset(ubx::kLayerRam, items) ==
        Bytes{0xB5, 0x62, 0x06, 0x8A, 0x14, 0x00,              // header, length 20
              0x00, 0x01, 0x00, 0x00,                          // version 0, RAM, reserved
              0x01, 0x00, 0x21, 0x30, 0x64, 0x00,              // CFG-RATE-MEAS = 100 ms
              0x02, 0x00, 0x74, 0x10, 0x00,                    // CFG-UART1OUTPROT-NMEA = 0
              0x07, 0x00, 0x91, 0x20, 0x01,                    // CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1
              0x9A, 0xFA});

  // layers are a bit field; only the value's own bytes are sent
  const ubx::ConfigItem wide[] = {{0x40520001, 115200}, {0x50000000, 0x0102030405060708}};
  Bytes frame = ubx::EncodeValset(ubx::kLayerRam | ubx::kLayerBbr, wide);
  REQUIRE(frame.size() == 8 + 4 + 8 + 12);
  CHECK(frame[7] == 0x03);
  CHECK(Bytes(frame.begin() + 10, frame.begin() + 18) ==
        Bytes{0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00});
  CHECK(Bytes(frame.begin() + 22, frame.begin() + 30) ==
        Bytes{0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01});
}

TEST_CASE("UBX-CFG-PRT byte layout", "[gnss][ubx]") {
  // M8 section 32.10.25: portID, reserved, txReady X2, mode X4, baudRate U4 (reserved on DDC),
  // inProtoMask X2 (UBX | NMEA | RTCM2), outProtoMask X2 (UBX), flags X2, reserved.
  CHECK(ubx::EncodeCfgPrtDdc(0x42) ==
        Bytes{0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
              0xA6, 0xC6});
  // UART1, mode 0x8C0 = 8 data bits, no parity, 1 stop bit; 9600 baud
  CHECK(ubx::EncodeCfgPrtUart1(9600) ==
        Bytes{0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00,
              0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x90, 0xA9});
}

TEST_CASE("UBX NAV-PVT encode and decode", "[gnss][ubx]") {
  ubx::NavPvt in = SamplePvt();
  auto payload = ubx::EncodeNavPvt(in);
  // spot-check offsets of M10 section 3.15.11
  CHECK(payload[4] == (2026 & 0xFF));
  CHECK(payload[5] == (2026 >> 8));
  CHECK(payload[20] == 3);
  CHECK(payload[23] == 11);
  CHECK(payload[28] == (410082560 & 0xFF));

  auto out = ubx::DecodeNavPvt(payload);
  REQUIRE(out.has_value());
  CHECK(out->iTow == in.iTow);
  CHECK(out->year == in.year);
  CHECK(out->month == in.month);
  CHECK(out->day == in.day);
  CHECK(out->hour == in.hour);
  CHECK(out->minute == in.minute);
  CHECK(out->second == in.second);
  CHECK(out->valid == in.valid);
  CHECK(out->nano == in.nano);
  CHECK(out->fixType == in.fixType);
  CHECK(out->flags == in.flags);
  CHECK(out->numSv == in.numSv);
  CHECK(out->lon == in.lon);
  CHECK(out->lat == in.lat);
  CHECK(out->height == in.height);
  CHECK(out->hMsl == in.hMsl);
  CHECK(out->hAcc == in.hAcc);
  CHECK(out->vAcc == in.vAcc);
  CHECK(out->velN == in.velN);
  CHECK(out->velE == in.velE);
  CHECK(out->velD == in.velD);
  CHECK(out->gSpeed == in.gSpeed);
  CHECK(out->headMot == in.headMot);
  CHECK(out->pDop == in.pDop);

  CHECK_FALSE(ubx::DecodeNavPvt(std::span<const uint8_t>{payload}.first(91)).has_value());
}

TEST_CASE("UBX parser handles split frames, garbage and bad checksums", "[gnss][ubx]") {
  auto pvt = ubx::EncodeNavPvt(SamplePvt());
  Bytes pvtFrame = ubx::Encode(ubx::kClassNav, ubx::kIdNavPvt, pvt);
  const uint8_t ackPayload[] = {ubx::kClassCfg, ubx::kIdCfgValset};
  Bytes ack = ubx::Encode(ubx::kClassAck, ubx::kIdAckAck, ackPayload);
  Bytes bad = pvtFrame;
  bad[50] ^= 0x01;  // payload bit flip: checksum mismatch

  Bytes stream{0xFF, 0xFF, 0xB5, 0xB5};  // idle I2C fill and a stray sync byte
  std::string_view nmea = "$GNTXT,01,01,02,u-blox*12\r\n";
  stream.insert(stream.end(), nmea.begin(), nmea.end());
  stream.insert(stream.end(), pvtFrame.begin(), pvtFrame.end());
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), {0xB5, 0x62, 0x01, 0x07, 0xFF, 0xFF});  // length 65535: dropped
  stream.push_back(0xFF);
  stream.insert(stream.end(), ack.begin(), ack.end());

  for (size_t split = 0; split <= stream.size(); ++split) {
    ubx::Parser parser;
    std::span<const uint8_t> all{stream};
    auto frames = PushAll(parser, all.first(split));
    auto rest = PushAll(parser, all.subspan(split));
    frames.insert(frames.end(), rest.begin(), rest.end());
    REQUIRE(frames.size() == 2);
    CHECK(frames[0].messageClass == ubx::kClassNav);
    CHECK(frames[0].messageId == ubx::kIdNavPvt);
    CHECK(frames[0].payload == Bytes(pvt.begin(), pvt.end()));
    CHECK(frames[1].messageClass == ubx::kClassAck);
    CHECK(frames[1].messageId == ubx::kIdAckAck);
    CHECK(frames[1].payload == Bytes(std::begin(ackPayload), std::end(ackPayload)));
    CHECK(parser.GetFrameCount() == 2);
    CHECK(parser.GetErrorCount() == 2);
  }

  // empty payload
  ubx::Parser parser;
  auto poll = ubx::Encode(ubx::kClassMon, ubx::kIdMonVer);
  auto frames = PushAll(parser, poll);
  REQUIRE(frames.size() == 1);
  CHECK(frames[0].payload.empty());
}

TEST_CASE("UBX-MON-VER protocol version", "[gnss][ubx]") {
  auto m10 = MonVer("PROTVER=34.10");
  auto v = ubx::ParseProtocolVersion(m10);
  REQUIRE(v.has_value());
  CHECK(*v > 34.09);
  CHECK(*v < 34.11);

  auto m8 = MonVer("PROTVER=18.00");
  v = ubx::ParseProtocolVersion(m8);
  REQUIRE(v.has_value());
  CHECK(*v > 17.99);
  CHECK(*v < 18.01);

  auto old = MonVer("PROTVER 15.00");  // early u-blox 8 firmware
  v = ubx::ParseProtocolVersion(old);
  REQUIRE(v.has_value());
  CHECK(*v > 14.99);
  CHECK(*v < 15.01);

  CHECK_FALSE(ubx::ParseProtocolVersion(MonVer("GPS;GLO;GAL;BDS")).has_value());
  CHECK_FALSE(ubx::ParseProtocolVersion(Bytes(40, 0)).has_value());
}
