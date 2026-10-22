// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/motor/DRV8317H.hpp"

#include <limits>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/hardware/discrete/DigitalOutput.hpp"
#include "prosdk/simulation/DIOSim.hpp"
#include "prosdk/simulation/SimDeviceSim.hpp"
#include "prosdk/simulation/SimHooks.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {

// INHA/B/C 0-2, INLA/B/C 3-5, Hall A/B/C 6-8, nSLEEP 9, nFAULT 10
DRV8317H::Pins TestPins() {
  DRV8317H::Pins p;
  p.inhA = 0;
  p.inhB = 1;
  p.inhC = 2;
  p.inlA = 3;
  p.inlB = 4;
  p.inlC = 5;
  p.hallA = 6;
  p.hallB = 7;
  p.hallC = 8;
  p.nSleep = 9;
  p.nFault = 10;
  return p;
}

struct Bridge {
  sim::DIOSim inh[3]{sim::DIOSim{0}, sim::DIOSim{1}, sim::DIOSim{2}};
  sim::DIOSim inl[3]{sim::DIOSim{3}, sim::DIOSim{4}, sim::DIOSim{5}};
  sim::DIOSim hall[3]{sim::DIOSim{6}, sim::DIOSim{7}, sim::DIOSim{8}};

  void SetHalls(int state) {
    for (int i = 0; i < 3; ++i) {
      hall[i].SetValue((state >> i) & 1);
    }
  }
  // 'P' = PWM (INL 1, INH 1), 'L' = low side (INL 1, INH 0), 'Z' = Hi-Z (INL 0, INH 0)
  bool Is(const char* phases) const {
    for (int i = 0; i < 3; ++i) {
      bool en = inl[i].GetValue();
      bool pwm = inh[i].GetValue();
      char c = !en ? (pwm ? '?' : 'Z') : (pwm ? 'P' : 'L');
      if (c != phases[i]) {
        return false;
      }
    }
    return true;
  }
};

}  // namespace

TEST_CASE("DRV8317H commutates from the Hall state", "[drv8317h]") {
  Bridge b;
  b.SetHalls(1);  // A
  DRV8317H m{TestPins()};
  CHECK(b.Is("ZZZ"));  // coast until the first throttle
  CHECK(m.GetHallState() == 1);
  CHECK_FALSE(sim::DIOSim{9}.GetIsInput());  // nSLEEP is an output, driven high: awake
  CHECK(sim::DIOSim{9}.GetValue());

  m.SetThrottle(0.5);  // state 1: C+ A-
  CHECK(b.Is("LZP"));
  b.hall[1].SetValue(true);  // state 3: C+ B-
  CHECK(b.Is("ZLP"));
  b.hall[0].SetValue(false);  // state 2: A+ B-
  CHECK(b.Is("PLZ"));
  b.hall[2].SetValue(true);  // state 6: A+ C-
  CHECK(b.Is("PZL"));
  b.hall[1].SetValue(false);  // state 4: B+ C-
  CHECK(b.Is("ZPL"));
  b.hall[0].SetValue(true);  // state 5: B+ A-
  CHECK(b.Is("LPZ"));

  m.SetThrottle(-0.5);  // reverse swaps the pair: A+ B-
  CHECK(b.Is("PLZ"));
  m.SetInverted(true);
  m.SetThrottle(0.5);
  CHECK(m.GetThrottle() == 0.5);
  CHECK(b.Is("PLZ"));
  sim::SimDeviceSim dev{"DRV8317H", 0};
  CHECK(dev.GetDouble("Throttle").Get() == -0.5);
  m.SetInverted(false);

  b.SetHalls(7);  // a disconnected sensor (pull-ups): no valid state, coast
  m.SetThrottle(0.5);
  CHECK(b.Is("ZZZ"));
  b.SetHalls(0);
  CHECK(b.Is("ZZZ"));
  b.SetHalls(2);
  CHECK(b.Is("PLZ"));

  CHECK(m.GetBrakeMode());
  m.SetThrottle(0.0);  // brake: all low sides on
  CHECK(b.Is("LLL"));
  m.SetBrakeMode(false);
  m.SetThrottle(0.0);  // coast
  CHECK(b.Is("ZZZ"));
  m.SetBrakeMode(true);
  m.SetThrottle(std::numeric_limits<double>::quiet_NaN());
  CHECK(m.GetThrottle() == 0.0);
  CHECK(b.Is("LLL"));
  m.SetThrottle(2.0);
  CHECK(m.GetThrottle() == 1.0);
  CHECK(b.Is("PLZ"));

  m.Disable();  // coast, the brake setting stays
  CHECK(m.GetThrottle() == 0.0);
  CHECK(b.Is("ZZZ"));
  CHECK(m.GetBrakeMode());
  m.StopMotor();
  CHECK(b.Is("ZZZ"));
}

TEST_CASE("DRV8317H speed and position from Hall steps", "[drv8317h]") {
  Bridge b;
  b.SetHalls(1);
  sim::PauseTiming();
  DRV8317H m{TestPins()};
  CHECK(m.GetRpm().value() == 0.0);  // no step yet
  CHECK(m.GetPosition().value() == 0.0);

  for (int state : {3, 2, 6}) {  // forward: C+B-, A+B-, A+C-
    sim::StepTiming(units::second_t{0.001});
    b.SetHalls(state);
  }
  // 3 steps of 1 ms: 1 electrical revolution per 6 ms = 10000 rpm at 1 pole pair
  CHECK(m.GetPosition().value() == 0.5);
  CHECK(m.GetRpm().value() == Catch::Approx(10000.0));
  m.SetPolePairs(2);
  CHECK(m.GetPolePairs() == 2);
  CHECK(m.GetRpm().value() == Catch::Approx(5000.0));
  CHECK(m.GetPosition().value() == 0.25);
  m.SetPolePairs(0);  // ignored
  CHECK(m.GetPolePairs() == 2);
  m.SetInverted(true);
  CHECK(m.GetRpm().value() == Catch::Approx(-5000.0));
  CHECK(m.GetPosition().value() == -0.25);
  m.SetInverted(false);

  sim::StepTiming(units::second_t{0.004});  // 4 ms since the last step: 1 step per 4 ms
  CHECK(m.GetRpm().value() == Catch::Approx(1250.0));
  sim::StepTiming(units::second_t{0.002});
  b.SetHalls(2);  // back one step
  CHECK(m.GetRpm().value() < 0.0);
  CHECK(m.GetPosition().value() == 1.0 / 6.0);

  sim::StepTiming(units::second_t{0.6});
  CHECK(m.GetRpm().value() == 0.0);

  // state 2 -> 0 (invalid, not counted) -> 4: a skipped step counts 2, at half the time
  sim::StepTiming(units::second_t{0.002});
  b.SetHalls(4);
  CHECK(m.GetHallState() == 4);
  CHECK(m.GetPosition().value() == 1.0 / 3.0);
  CHECK(m.GetRpm().value() == Catch::Approx(60.0 / (6 * 2 * 0.301)));
  sim::ResumeTiming();
}

TEST_CASE("DRV8317H commutation table, fault and pins", "[drv8317h]") {
  Bridge b;
  b.SetHalls(1);
  DRV8317H m{TestPins()};

  DRV8317H::CommutationTable bad = platform::hal::kDefaultSixStepTable;
  bad[3] = bad[1];  // one phase pair twice
  CHECK_FALSE(m.SetCommutationTable(bad));
  bad = platform::hal::kDefaultSixStepTable;
  bad[1] = {1, 1};
  CHECK_FALSE(m.SetCommutationTable(bad));
  bad = platform::hal::kDefaultSixStepTable;
  bad[1] = {};  // only five states
  CHECK_FALSE(m.SetCommutationTable(bad));

  // the default table with the directions swapped
  DRV8317H::CommutationTable swapped = platform::hal::kDefaultSixStepTable;
  for (auto& d : swapped) {
    d = {d.low, d.high};
  }
  CHECK(m.SetCommutationTable(swapped));
  m.SetThrottle(0.5);  // state 1: A+ C-
  CHECK(b.Is("PZL"));

  sim::DIOSim fault{10};
  fault.SetValue(false);
  CHECK(m.IsFault());
  fault.SetValue(true);
  CHECK_FALSE(m.IsFault());

  CHECK(m.GetDescription() == "DRV8317H 0/1/2");

  DRV8317H::Pins other = TestPins();
  other.inhA = 20;  // INHB-C, INL, Halls taken by m
  other.nSleep = -1;
  other.nFault = -1;
  CHECK_THROWS_AS(DRV8317H{other}, RuntimeError);

  DRV8317H::Pins twice{20, 21, 22, 23, 24, 25, 26, 27, 20};
  CHECK_THROWS_AS(DRV8317H{twice}, RuntimeError);

  DigitalOutput dio{26};
  DRV8317H::Pins dioTaken{20, 21, 22, 23, 24, 25, 26, 27, 28};
  CHECK_THROWS_AS(DRV8317H{dioTaken}, RuntimeError);
  DRV8317H::Pins outOfRange{20, 21, 22, 23, 24, 25, 27, 28, 31};
  CHECK_THROWS_AS(DRV8317H{outOfRange}, RuntimeError);

  DRV8317H::Pins free{20, 21, 22, 23, 24, 25, 27, 28, 29};
  sim::DIOSim{27}.SetValue(false);
  sim::DIOSim{28}.SetValue(false);
  sim::DIOSim{29}.SetValue(false);
  DRV8317H second{free};  // a second motor on free pins
  CHECK(second.GetHallState() == 0);
}
