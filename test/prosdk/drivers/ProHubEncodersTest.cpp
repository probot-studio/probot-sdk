// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/encoder/ProHubEncoders.hpp"

#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/simulation/SimHooks.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

TEST_CASE("ProHubEncoders parses the 108-byte frame", "[prohub][encoder]") {
  ProHubEncoders enc{SerialPort::Port::ONBOARD, ProHubEncoders::kDefaultBaudRate, 0_ms};
  CHECK_FALSE(enc.GetFrame().has_value());
  CHECK_FALSE(enc.IsConnected());
  CHECK(enc.GetTicks(0) == 0);

  ProHubEncoders::Frame f;
  f.seq = 7;
  f.timestampUs = 123456;
  f.dtUs = 10000;
  f.totalTicks[0] = 1000;
  f.totalTicks[3] = -5;
  f.totalTicks[7] = int64_t{1} << 40;
  f.deltaTicks[0] = 50;
  f.deltaTicks[3] = -2;
  auto bytes = ProHubEncoders::Encode(f);
  CHECK(bytes.size() == 108);

  // garbage, a lone 0xAA, then the frame split over two calls
  std::vector<uint8_t> stream{0x00, 0x12, 0xAA, 0x13, 0xAA};
  stream.insert(stream.end(), bytes.begin(), bytes.end());
  enc.Feed({stream.data(), 40});
  CHECK(enc.GetFrameCount() == 0);
  enc.Feed({stream.data() + 40, stream.size() - 40});
  CHECK(enc.GetFrameCount() == 1);
  CHECK(enc.GetBytesReceived() == stream.size());
  CHECK(enc.IsConnected());
  auto got = enc.GetFrame();
  REQUIRE(got.has_value());
  CHECK(got->seq == 7);
  CHECK(got->timestampUs == 123456);
  CHECK(got->totalTicks == f.totalTicks);
  CHECK(got->deltaTicks == f.deltaTicks);
  CHECK(enc.GetTicks(3) == -5);
  CHECK(enc.GetTicksPerSecond(0) == 5000.0);
  CHECK(enc.GetTicksPerSecond(3) == -200.0);

  auto bad = bytes;
  bad[20] ^= 0x01;
  enc.Feed(bad);
  CHECK(enc.GetCrcErrors() == 1);
  CHECK(enc.GetFrameCount() == 1);
  CHECK_THROWS_AS(enc.GetTicks(8), RuntimeError);
}

TEST_CASE("ProHubEncoder distance and rate", "[prohub][encoder]") {
  ProHubEncoders enc{SerialPort::Port::ONBOARD, ProHubEncoders::kDefaultBaudRate, 0_ms};
  ProHubEncoders::Frame f;
  f.dtUs = 10000;
  f.totalTicks[2] = 1000;
  enc.Feed(ProHubEncoders::Encode(f));

  ProHubEncoder e{enc, 2};
  CHECK(e.GetIndex() == 2);
  CHECK(e.Get() == 0);  // zeroed at construction
  f.totalTicks[2] = 1100;
  f.deltaTicks[2] = 50;
  enc.Feed(ProHubEncoders::Encode(f));
  CHECK(e.Get() == 100);
  e.SetDistancePerPulse(0.01);
  CHECK(e.GetDistancePerPulse() == 0.01);
  CHECK(e.GetDistance() == 1.0);
  CHECK(e.GetRate() == 50.0);
  e.SetReverseDirection(true);
  CHECK(e.Get() == -100);
  CHECK(e.GetRate() == -50.0);
  e.Reset();
  CHECK(e.Get() == 0);
  CHECK(e.GetTelemetryType() == "Encoder");
  CHECK_THROWS_AS((ProHubEncoder{enc, -1}), RuntimeError);
}

TEST_CASE("ProHubEncoders reports a silent stream as disconnected", "[prohub][encoder]") {
  sim::PauseTiming();
  ProHubEncoders enc{SerialPort::Port::ONBOARD, ProHubEncoders::kDefaultBaudRate, 0_ms};
  ProHubEncoders::Frame f;
  enc.Feed(ProHubEncoders::Encode(f));
  CHECK(enc.IsConnected());
  sim::StepTiming(90_ms);
  CHECK(enc.IsConnected());
  sim::StepTiming(20_ms);
  CHECK_FALSE(enc.IsConnected());  // no frame for more than 100 ms
  enc.Feed(ProHubEncoders::Encode(f));
  CHECK(enc.IsConnected());
  sim::ResumeTiming();
}
