// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include <stdint.h>

#include <array>
#include <optional>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/drivers/bus/PulseCapture.hpp"
#include "prosdk/drivers/bus/PulseTransmitter.hpp"
#include "prosdk/simulation/PulseIOSim.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {

std::vector<Pulse> ReadAll(PulseCapture& rx) {
  std::array<Pulse, 64> buf{};
  std::vector<Pulse> all;
  for (int n; (n = rx.Read(buf)) > 0;) {
    all.insert(all.end(), buf.begin(), buf.begin() + n);
  }
  return all;
}

}  // namespace

TEST_CASE("PulseTransmitter sends trains on its carrier", "[pulseio]") {
  PulseTransmitter tx{40, units::hertz_t{38000}};
  CHECK(tx.GetChannel() == 40);
  CHECK(tx.GetCarrier() == units::hertz_t{38000});
  uint32_t hz = 0;
  double duty = 0.0;
  REQUIRE(sim::GetPulseTxCarrier(40, &hz, &duty));
  CHECK(hz == 38000u);
  CHECK(duty == PulseTransmitter::kDefaultCarrierDuty);

  std::vector<std::vector<Pulse>> sent;
  int id = sim::OnPulsesSent(40, [&](std::span<const Pulse> p, uint32_t carrier) {
    CHECK(carrier == 38000u);
    sent.emplace_back(p.begin(), p.end());
  });
  // NEC leader: 9 ms burst, 4.5 ms space, then a 562 us burst
  std::vector<Pulse> leader{{true, 9000}, {false, 4500}, {true, 562}};
  CHECK(tx.Send(leader));
  CHECK(tx.WaitSent(units::second_t{0}));
  REQUIRE(sent.size() == 1);
  CHECK(sent[0] == leader);

  // out of range: nothing is sent
  CHECK_FALSE(tx.Send(std::vector<Pulse>{{true, 0}}));
  CHECK_FALSE(tx.Send(std::vector<Pulse>{{true, platform::hal::kMaxPulseUs + 1}}));
  CHECK_FALSE(tx.Send(std::vector<Pulse>(platform::hal::kMaxPulsesPerSend + 1, {true, 10})));
  CHECK(tx.Send(std::vector<Pulse>(platform::hal::kMaxPulsesPerSend, {true, 10})));
  CHECK(sent.size() == 2);
  sim::CancelPulsesSent(id);
  CHECK(tx.Send(leader));
  CHECK(sent.size() == 2);
}

TEST_CASE("PulseTransmitter without a carrier and argument errors", "[pulseio]") {
  PulseTransmitter plain{41};
  uint32_t hz = 1;
  double duty = 1.0;
  REQUIRE(sim::GetPulseTxCarrier(41, &hz, &duty));
  CHECK(hz == 0u);
  CHECK(plain.GetCarrier() == units::hertz_t{0});

  CHECK_THROWS_AS((PulseTransmitter{42, units::hertz_t{1999}}), RuntimeError);  // < 2 kHz
  CHECK_NOTHROW((PulseTransmitter{42, units::hertz_t{2000}, 0.9}));  // the lowest carrier
  CHECK_THROWS_AS((PulseTransmitter{42, units::hertz_t{1000001}}), RuntimeError);
  CHECK_THROWS_AS((PulseTransmitter{42, units::hertz_t{38000}, 0.95}), RuntimeError);
  CHECK_THROWS_AS((PulseTransmitter{42, units::hertz_t{-5}}), RuntimeError);
  CHECK_THROWS_AS((PulseTransmitter{49}), RuntimeError);
  CHECK_THROWS_AS((PulseTransmitter{41}), RuntimeError);  // in use
  CHECK_THROWS_AS((PulseCapture{41}), RuntimeError);      // in use by the transmitter
  CHECK_FALSE(sim::GetPulseTxCarrier(42, &hz, &duty));
}

TEST_CASE("Pulse channel limits", "[pulseio]") {
  {
    PulseTransmitter a{40};
    PulseTransmitter b{41};
    PulseTransmitter c{42};
    CHECK_THROWS_AS((PulseTransmitter{43}), RuntimeError);  // 3 RMT TX channels
  }
  PulseTransmitter again{43};  // freed on destruction
  PulseCapture r0{44};
  PulseCapture r1{45};
  PulseCapture r2{46};
  {
    PulseCapture r3{47};
    CHECK_THROWS_AS((PulseCapture{48}), RuntimeError);  // 4 RMT RX channels
  }
  PulseCapture r3{48};
  CHECK(r3.GetChannel() == 48);
  CHECK_THROWS_AS((PulseCapture{38, units::microsecond_t{5}}), RuntimeError);       // idle < 10 us
  CHECK_THROWS_AS((PulseCapture{38, units::microsecond_t{40000}}), RuntimeError);   // > 32767 us
}

TEST_CASE("PulseCapture splits the line into bursts", "[pulseio]") {
  PulseCapture rx{44, units::microsecond_t{12000}};  // longer than the 9 ms leader
  CHECK_FALSE(sim::InjectPulses(45, std::vector<Pulse>{{true, 10}}));

  // idle high (pull-up), a frame, then idle high again
  std::vector<Pulse> line{{true, 20000}, {false, 9000}, {true, 4500}, {false, 560},
                          {true, 1690}, {false, 560}, {true, 20000}};
  REQUIRE(sim::InjectPulses(44, line));
  std::vector<Pulse> want{{false, 9000}, {true, 4500}, {false, 560},
                          {true, 1690}, {false, 560}, {true, 0}};
  CHECK(ReadAll(rx) == want);
  CHECK(rx.GetDropped() == 0);

  // more idle: nothing; equal neighbouring levels merge
  REQUIRE(sim::InjectPulses(44, std::vector<Pulse>{{true, 8000},
                                                   {false, 300},
                                                   {false, 200},
                                                   {true, 15000}}));
  CHECK(ReadAll(rx) == std::vector<Pulse>{{false, 500}, {true, 0}});
}

TEST_CASE("PulseCapture glitch filter and overflow", "[pulseio]") {
  PulseCapture rx{46, units::microsecond_t{1000}, units::microsecond_t{3}};
  REQUIRE(sim::InjectPulses(46, std::vector<Pulse>{{true, 2000},
                                                   {false, 100},
                                                   {true, 2},  // filtered: low lasts 2 us longer
                                                   {false, 50},
                                                   {true, 1500}}));
  CHECK(ReadAll(rx) == std::vector<Pulse>{{false, 152}, {true, 0}});
  CHECK_THROWS_AS((PulseCapture{47, units::microsecond_t{1000}, units::microsecond_t{4}}),
                  RuntimeError);

  std::vector<Pulse> burst;
  for (int i = 0; i < 150; ++i) {
    burst.push_back({false, 100});
    burst.push_back({true, 100});
  }
  REQUIRE(sim::InjectPulses(46, burst));  // 300 pulses into a 256-pulse buffer
  CHECK(rx.GetDropped() == 300 - platform::hal::kRxBufferPulses);
  CHECK(ReadAll(rx).size() == static_cast<size_t>(platform::hal::kRxBufferPulses));
}

TEST_CASE("PulseTransmitter loops back into a PulseCapture", "[pulseio]") {
  PulseTransmitter tx{40};
  PulseCapture rx{41, units::microsecond_t{2000}};
  int id = sim::OnPulsesSent(40, [](std::span<const Pulse> p, uint32_t) {
    sim::InjectPulses(41, p);
    sim::InjectPulses(41, std::vector<Pulse>{{false, 3000}});  // the line returns low
  });
  std::vector<Pulse> word{{true, 300}, {false, 100}, {true, 100}, {false, 300}, {true, 300}};
  CHECK(tx.Send(word));
  std::vector<Pulse> want = word;
  want.push_back({false, 0});
  CHECK(ReadAll(rx) == want);
  sim::CancelPulsesSent(id);
}
