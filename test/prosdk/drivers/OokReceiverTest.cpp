// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/radio/OokReceiver.hpp"

#include <stdint.h>

#include <optional>
#include <string_view>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/simulation/PulseIOSim.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {

// What the receiver's AGC outputs with no transmitter: short random levels, ending low.
const std::vector<Pulse> kNoise{{true, 120}, {false, 80}, {true, 300}, {false, 950},
                                {true, 60},  {false, 400}, {true, 45}, {false, 700}};

void AddSync(std::vector<Pulse>& line, uint32_t unit) {
  line.push_back({true, unit});
  line.push_back({false, 31 * unit});
}

// One frame of `bits` bits, MSB first: 1 = 3 units high + 1 low, 0 = 1 high + 3 low. `jitter`
// lengthens or shortens every high (alternately) and the low after it takes up the difference,
// as a receiver's slicer does.
void AddBits(std::vector<Pulse>& line, uint32_t value, int bits, uint32_t unit, int jitter = 0) {
  for (int b = bits - 1; b >= 0; --b) {
    bool one = ((value >> b) & 1u) != 0;
    int j = b % 2 == 0 ? jitter : -jitter;
    int high = static_cast<int>((one ? 3 : 1) * unit) + j;
    int low = static_cast<int>((one ? 1 : 3) * unit) - j;
    line.push_back({true, static_cast<uint32_t>(high)});
    line.push_back({false, static_cast<uint32_t>(low)});
  }
}

// EV1527 style: the sync before every frame; a last sync so that the last frame counts.
std::vector<Pulse> Ev1527(uint32_t value, int bits, uint32_t unit, int frames, int jitter = 0) {
  std::vector<Pulse> line = kNoise;
  for (int f = 0; f < frames; ++f) {
    AddSync(line, unit);
    AddBits(line, value, bits, unit, jitter);
  }
  AddSync(line, unit);
  line.insert(line.end(), kNoise.begin(), kNoise.end());
  return line;
}

// PT2262 tri-state digits ('0' = 00, '1' = 11, 'F' = 01) as a 24-bit value
uint32_t Pt2262Value(std::string_view digits) {
  uint32_t v = 0;
  for (char d : digits) {
    v = (v << 2) | (d == '1' ? 3u : d == 'F' ? 1u : 0u);
  }
  return v;
}

std::vector<OokCode> PollAll(OokReceiver& rx) {
  std::vector<OokCode> codes;
  while (auto c = rx.Poll()) {
    codes.push_back(*c);
  }
  return codes;
}

}  // namespace

TEST_CASE("OokReceiver decodes EV1527 frames after two in a row", "[ookreceiver]") {
  OokReceiver rx{40};
  CHECK(rx.GetChannel() == 40);
  CHECK_FALSE(rx.Poll().has_value());

  uint32_t code = (0x5A3C9u << 4) | 0x3;  // 20-bit ID, keys D0 and D1
  REQUIRE(sim::InjectPulses(40, Ev1527(code, 24, 350, 4)));
  auto codes = PollAll(rx);
  // 4 frames: the first is only remembered, the second confirms it, then the key is held
  REQUIRE(codes.size() == 3);
  for (size_t i = 0; i < codes.size(); ++i) {
    CHECK(codes[i].value == code);
    CHECK(codes[i].bits == 24);
    CHECK(codes[i].pulseLength.value() == Catch::Approx(350.0));
    CHECK(codes[i].repeat == (i > 0));
  }

  // a new press (noise in between) starts over
  REQUIRE(sim::InjectPulses(40, Ev1527(code, 24, 350, 2)));
  codes = PollAll(rx);
  REQUIRE(codes.size() == 1);
  CHECK_FALSE(codes[0].repeat);

  auto recent = rx.GetRecentPulses();
  REQUIRE_FALSE(recent.empty());
  CHECK(recent.size() <= static_cast<size_t>(OokReceiver::kRecentPulses));
  CHECK(recent.back() == kNoise.back());
}

TEST_CASE("OokReceiver decodes PT2262 frames with jitter", "[ookreceiver]") {
  OokReceiver rx{41};
  uint32_t value = Pt2262Value("0F1F00001101");
  // PT2262 sends the sync after each word; the line then stays low (a 0 us end marker)
  std::vector<Pulse> line = kNoise;
  for (int f = 0; f < 3; ++f) {
    AddBits(line, value, 24, 300, 100);
    AddSync(line, 300);
  }
  line.back().us = 40000;  // longer than the 32.767 ms capture idle time
  REQUIRE(sim::InjectPulses(41, line));
  auto codes = PollAll(rx);
  // the first word follows noise (no sync before it); the second and third match
  REQUIRE(codes.size() == 1);
  CHECK(codes[0].value == value);
  CHECK(codes[0].bits == 24);
  CHECK(codes[0].pulseLength.value() == Catch::Approx(300.0));
  CHECK_FALSE(codes[0].repeat);
}

TEST_CASE("OokReceiver frame lengths and pulse lengths", "[ookreceiver]") {
  OokReceiver rx{42};
  REQUIRE(sim::InjectPulses(42, Ev1527(0xDEADBEEF, 32, 200, 2)));
  auto codes = PollAll(rx);
  REQUIRE(codes.size() == 1);
  CHECK(codes[0].value == 0xDEADBEEFu);
  CHECK(codes[0].bits == 32);
  CHECK(codes[0].pulseLength.value() == Catch::Approx(200.0));

  REQUIRE(sim::InjectPulses(42, Ev1527(0x5A, 8, 600, 2)));
  codes = PollAll(rx);
  REQUIRE(codes.size() == 1);
  CHECK(codes[0].value == 0x5Au);
  CHECK(codes[0].bits == 8);

  // too few bits
  REQUIRE(sim::InjectPulses(42, Ev1527(0x5, 4, 350, 3)));
  CHECK_FALSE(rx.Poll().has_value());
  // a short pulse of 40 us (below 50 us), with 2 ms sync gaps so that the frames are found;
  // the same frames at 60 us are accepted (the limit is the pulse length, not the gap)
  auto withShortGaps = [](uint32_t unit) {
    std::vector<Pulse> line = kNoise;
    for (int f = 0; f < 4; ++f) {
      line.push_back({true, unit});
      line.push_back({false, 2000});
      AddBits(line, 0xABCDE, 20, unit);
    }
    line.push_back({true, unit});
    line.push_back({false, 2000});
    line.insert(line.end(), kNoise.begin(), kNoise.end());
    return line;
  };
  REQUIRE(sim::InjectPulses(42, withShortGaps(40)));
  CHECK_FALSE(rx.Poll().has_value());
  REQUIRE(sim::InjectPulses(42, withShortGaps(60)));
  codes = PollAll(rx);
  REQUIRE(codes.size() == 3);
  CHECK(codes[0].value == 0xABCDEu);
  CHECK(codes[0].bits == 20);
  CHECK(codes[0].pulseLength.value() == Catch::Approx(60.0));
}

TEST_CASE("OokReceiver rejects single and broken frames", "[ookreceiver]") {
  OokReceiver rx{43};
  // one frame alone is never accepted (noise can look like one)
  REQUIRE(sim::InjectPulses(43, Ev1527(0x123456, 24, 350, 1)));
  CHECK_FALSE(rx.Poll().has_value());

  // a bit with equal high and low breaks the run: A, broken, A gives no confirmed pair
  std::vector<Pulse> line = kNoise;
  AddSync(line, 350);
  AddBits(line, 0x123456, 24, 350);
  AddSync(line, 350);
  AddBits(line, 0x123456, 24, 350);
  line[line.size() - 10] = {true, 700};  // 2 units high
  line[line.size() - 9] = {false, 700};  // 2 units low
  AddSync(line, 350);
  AddBits(line, 0x123456, 24, 350);
  AddSync(line, 350);
  line.insert(line.end(), kNoise.begin(), kNoise.end());
  REQUIRE(sim::InjectPulses(43, line));
  CHECK_FALSE(rx.Poll().has_value());

  CHECK_THROWS_AS((OokReceiver{43}), RuntimeError);  // pin in use
}
