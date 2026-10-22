// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/radio/IRTransceiver.hpp"

#include <stdint.h>

#include <optional>
#include <span>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/simulation/PulseIOSim.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {

// NEC frame as carrier levels: 9 ms leader, 4.5 ms space, 32 bits LSB first, stop burst.
std::vector<Pulse> NecTrain(uint32_t word) {
  std::vector<Pulse> train{{true, 9000}, {false, 4500}};
  for (int bit = 0; bit < 32; ++bit) {
    train.push_back({true, 562});
    train.push_back({false, ((word >> bit) & 1u) != 0 ? 1687u : 562u});
  }
  train.push_back({true, 562});
  return train;
}

uint32_t NecWord(uint8_t a0, uint8_t a1, uint8_t command) {
  return a0 | (uint32_t{a1} << 8) | (uint32_t{command} << 16) |
         (uint32_t{static_cast<uint8_t>(~command)} << 24);
}

// What the IRM-H638T outputs for a carrier train: low while the carrier is on, idle high around
// it. `markDelta` lengthens every burst (and shortens every space) as the receiver distorts them.
std::vector<Pulse> ReceiverOutput(std::span<const Pulse> carrier, int markDelta = 0) {
  std::vector<Pulse> line{{true, 20000}};
  for (const Pulse& p : carrier) {
    int us = static_cast<int>(p.us) + (p.level ? markDelta : -markDelta);
    line.push_back({!p.level, static_cast<uint32_t>(us)});
  }
  line.push_back({true, 20000});
  return line;
}

}  // namespace

TEST_CASE("IRTransceiver sends NEC frames and decodes its own echo", "[irtransceiver]") {
  IRTransceiver ir{40, 41};
  uint32_t hz = 0;
  double duty = 0.0;
  REQUIRE(sim::GetPulseTxCarrier(40, &hz, &duty));
  CHECK(hz == 38000u);
  CHECK(duty == Catch::Approx(IRTransceiver::kCarrierDuty));

  std::vector<std::vector<Pulse>> sent;
  int id = sim::OnPulsesSent(40, [&](std::span<const Pulse> p, uint32_t) {
    sent.emplace_back(p.begin(), p.end());
    sim::InjectPulses(41, ReceiverOutput(p));  // the receiver sees the board's own LED
  });
  CHECK_FALSE(ir.Poll().has_value());

  REQUIRE(ir.SendNec(0x12, 0x34));
  REQUIRE(sent.size() == 1);
  CHECK(sent[0] == NecTrain(NecWord(0x12, 0xED, 0x34)));
  CHECK(sent[0].size() == 67u);
  CHECK(sent[0][3] == Pulse{false, 562});   // address bit 0 = 0
  CHECK(sent[0][5] == Pulse{false, 1687});  // address bit 1 = 1
  auto frame = ir.Poll();
  REQUIRE(frame.has_value());
  CHECK(*frame == IrFrame{IrProtocol::kNec, 0x12, 0x34, false});
  CHECK_FALSE(ir.Poll().has_value());
  CHECK(ir.GetLastPulses() == sent[0]);

  // a held key: repeat codes carry the frame's address and command
  REQUIRE(ir.SendNecRepeat());
  REQUIRE(sent.size() == 2);
  CHECK(sent[1] == std::vector<Pulse>{{true, 9000}, {false, 2250}, {true, 562}});
  frame = ir.Poll();
  REQUIRE(frame.has_value());
  CHECK(*frame == IrFrame{IrProtocol::kNec, 0x12, 0x34, true});
  REQUIRE(ir.SendNecRepeat());
  frame = ir.Poll();
  REQUIRE(frame.has_value());
  CHECK(frame->repeat);

  // raw trains pass through unchanged
  std::vector<Pulse> raw{{true, 600}, {false, 600}, {true, 1200}};
  REQUIRE(ir.SendRaw(raw));
  CHECK(sent.back() == raw);
  CHECK_FALSE(ir.Poll().has_value());  // not NEC
  CHECK(ir.GetLastPulses() == raw);
  CHECK_FALSE(ir.SendRaw(std::vector<Pulse>{{true, 0}}));  // out of range, nothing sent
  sim::CancelPulsesSent(id);
}

TEST_CASE("IRTransceiver decodes distorted and extended NEC frames", "[irtransceiver]") {
  IRTransceiver ir{-1, 42};

  // bursts 200 us long, spaces 200 us short (the receiver's worst case for a 600 us burst)
  REQUIRE(sim::InjectPulses(42, ReceiverOutput(NecTrain(NecWord(0x00, 0xFF, 0x45)), 200)));
  auto frame = ir.Poll();
  REQUIRE(frame.has_value());
  CHECK(*frame == IrFrame{IrProtocol::kNec, 0x00, 0x45, false});

  // the opposite distortion, and a 16-bit address (second byte not the inverse of the first)
  REQUIRE(sim::InjectPulses(42, ReceiverOutput(NecTrain(NecWord(0x34, 0x12, 0x9A)), -200)));
  frame = ir.Poll();
  REQUIRE(frame.has_value());
  CHECK(*frame == IrFrame{IrProtocol::kNecExtended, 0x1234, 0x9A, false});

  // a wrong command inverse is dropped, and the repeat code after it has no frame to repeat
  uint32_t bad = NecWord(0x01, 0xFE, 0x02) ^ 0x80000000u;
  REQUIRE(sim::InjectPulses(42, ReceiverOutput(NecTrain(bad))));
  CHECK_FALSE(ir.Poll().has_value());
  CHECK(ir.GetLastPulses().size() == 67u);
  std::vector<Pulse> repeat{{true, 9000}, {false, 2250}, {true, 562}};
  REQUIRE(sim::InjectPulses(42, ReceiverOutput(repeat)));
  CHECK_FALSE(ir.Poll().has_value());

  // several frames between two polls are queued in order
  std::vector<Pulse> line;
  for (uint8_t cmd : {0x10, 0x11, 0x12}) {
    auto one = ReceiverOutput(NecTrain(NecWord(0x07, 0xF8, cmd)));
    line.insert(line.end(), one.begin(), one.end());
  }
  REQUIRE(sim::InjectPulses(42, line));
  for (uint8_t cmd : {0x10, 0x11, 0x12}) {
    frame = ir.Poll();
    REQUIRE(frame.has_value());
    CHECK(*frame == IrFrame{IrProtocol::kNec, 0x07, cmd, false});
  }
  CHECK_FALSE(ir.Poll().has_value());

  // a bit space between the 0 and 1 ranges spoils the frame
  auto broken = NecTrain(NecWord(0x07, 0xF8, 0x10));
  broken[9].us = 1100;
  REQUIRE(sim::InjectPulses(42, ReceiverOutput(broken)));
  CHECK_FALSE(ir.Poll().has_value());
}

TEST_CASE("IRTransceiver with one pin", "[irtransceiver]") {
  IRTransceiver rxOnly{-1, 43};
  CHECK_FALSE(rxOnly.SendNec(1, 2));
  CHECK_FALSE(rxOnly.SendNecRepeat());

  IRTransceiver txOnly{44, -1};
  CHECK_FALSE(txOnly.Poll().has_value());
  CHECK(txOnly.SendNec(1, 2));
  CHECK(txOnly.GetLastPulses().empty());

  CHECK_THROWS_AS((IRTransceiver{44, 45}), RuntimeError);  // TX pin in use
  CHECK_THROWS_AS((IRTransceiver{45, 43}), RuntimeError);  // RX pin in use
}
