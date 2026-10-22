// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/REVPHSim.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "callback_helpers/TestCallbackHelpers.hpp"
#include "prosdk/hardware/pneumatic/DoubleSolenoid.hpp"
#include "prosdk/hardware/pneumatic/PneumaticHub.hpp"

namespace prosdk::sim {

TEST_CASE("REVPHSimTest InitializedCallback", "[wpilibc][simulation]") {
  REVPHSim sim;

  sim.ResetData();
  CHECK_FALSE(sim.GetInitialized());

  BooleanCallback callback;
  auto cb = sim.RegisterInitializedCallback(callback.GetCallback(), false);

  PneumaticHub ph{CANPort::CAN_S0};
  CHECK(sim.GetInitialized());
  CHECK(callback.WasTriggered());
  CHECK(callback.GetLastValue());
}

TEST_CASE("REVPHSimTest SolenoidOutput", "[wpilibc][simulation]") {
  PneumaticHub ph{CANPort::CAN_S0};
  REVPHSim sim(ph);
  sim.ResetData();

  DoubleSolenoid doubleSolenoid{CANPort::CAN_S0, 1,
                                prosdk::PneumaticsModuleType::REV_PH, 3, 4};

  BooleanCallback callback3;
  BooleanCallback callback4;
  auto cb3 =
      sim.RegisterSolenoidOutputCallback(3, callback3.GetCallback(), false);
  auto cb4 =
      sim.RegisterSolenoidOutputCallback(4, callback4.GetCallback(), false);

  callback3.Reset();
  callback4.Reset();
  doubleSolenoid.Set(DoubleSolenoid::REVERSE);
  CHECK_FALSE(callback3.WasTriggered());
  CHECK_FALSE(callback3.GetLastValue());
  CHECK(callback4.WasTriggered());
  CHECK(callback4.GetLastValue());
  CHECK_FALSE(sim.GetSolenoidOutput(3));
  CHECK(sim.GetSolenoidOutput(4));
  CHECK(0b00010000 == ph.GetSolenoids());
  CHECK(0b00010000 == sim.GetAllSolenoidOutputs());

  callback3.Reset();
  callback4.Reset();
  doubleSolenoid.Set(DoubleSolenoid::FORWARD);
  CHECK(callback3.WasTriggered());
  CHECK(callback3.GetLastValue());
  CHECK(callback4.WasTriggered());
  CHECK_FALSE(callback4.GetLastValue());
  CHECK(sim.GetSolenoidOutput(3));
  CHECK_FALSE(sim.GetSolenoidOutput(4));
  CHECK(0b00001000 == ph.GetSolenoids());
  CHECK(0b00001000 == sim.GetAllSolenoidOutputs());

  callback3.Reset();
  callback4.Reset();
  doubleSolenoid.Set(DoubleSolenoid::OFF);
  CHECK(callback3.WasTriggered());
  CHECK_FALSE(callback3.GetLastValue());
  CHECK_FALSE(callback4.WasTriggered());
  CHECK_FALSE(callback4.GetLastValue());
  CHECK_FALSE(sim.GetSolenoidOutput(3));
  CHECK_FALSE(sim.GetSolenoidOutput(4));
  CHECK(0b00000000 == ph.GetSolenoids());
  CHECK(0b00000000 == sim.GetAllSolenoidOutputs());
}

TEST_CASE("REVPHSimTest SetCompressorOn", "[wpilibc][simulation]") {
  PneumaticHub ph{CANPort::CAN_S0};
  REVPHSim sim(ph);
  sim.ResetData();

  BooleanCallback callback;
  auto cb = sim.RegisterCompressorOnCallback(callback.GetCallback(), false);

  CHECK_FALSE(ph.GetCompressor());
  CHECK_FALSE(ph.GetCompressor());
  sim.SetCompressorOn(true);
  CHECK(sim.GetCompressorOn());
  CHECK(ph.GetCompressor());
  CHECK(callback.WasTriggered());
  CHECK(callback.GetLastValue());
}

TEST_CASE("REVPHSimTest SetEnableDigital", "[wpilibc][simulation]") {
  PneumaticHub ph{CANPort::CAN_S0};
  REVPHSim sim(ph);
  sim.ResetData();

  EnumCallback callback;
  auto cb =
      sim.RegisterCompressorConfigTypeCallback(callback.GetCallback(), false);

  ph.DisableCompressor();
  CHECK(ph.GetCompressorConfigType() == CompressorConfigType::kDisabled);

  ph.EnableCompressorDigital();
  CHECK(sim.GetCompressorConfigType() ==
        static_cast<int>(CompressorConfigType::kDigital));
  CHECK(ph.GetCompressorConfigType() == CompressorConfigType::kDigital);
  CHECK(callback.WasTriggered());
  CHECK(callback.GetLastValue() ==
        static_cast<int>(CompressorConfigType::kDigital));
}

TEST_CASE("REVPHSimTest SetEnableAnalog", "[wpilibc][simulation]") {
  PneumaticHub ph{CANPort::CAN_S0};
  REVPHSim sim(ph);
  sim.ResetData();

  EnumCallback callback;
  auto cb =
      sim.RegisterCompressorConfigTypeCallback(callback.GetCallback(), false);

  ph.DisableCompressor();
  CHECK(ph.GetCompressorConfigType() == CompressorConfigType::kDisabled);

  ph.EnableCompressorAnalog(1_psi, 2_psi);
  CHECK(sim.GetCompressorConfigType() ==
        static_cast<int>(CompressorConfigType::kAnalog));
  CHECK(ph.GetCompressorConfigType() == CompressorConfigType::kAnalog);
  CHECK(callback.WasTriggered());
  CHECK(callback.GetLastValue() ==
        static_cast<int>(CompressorConfigType::kAnalog));
}

TEST_CASE("REVPHSimTest SetEnableHybrid", "[wpilibc][simulation]") {
  PneumaticHub ph{CANPort::CAN_S0};
  REVPHSim sim(ph);
  sim.ResetData();

  EnumCallback callback;
  auto cb =
      sim.RegisterCompressorConfigTypeCallback(callback.GetCallback(), false);

  ph.DisableCompressor();
  CHECK(ph.GetCompressorConfigType() == CompressorConfigType::kDisabled);

  ph.EnableCompressorHybrid(1_psi, 2_psi);
  CHECK(sim.GetCompressorConfigType() ==
        static_cast<int>(CompressorConfigType::kHybrid));
  CHECK(ph.GetCompressorConfigType() == CompressorConfigType::kHybrid);
  CHECK(callback.WasTriggered());
  CHECK(callback.GetLastValue() ==
        static_cast<int>(CompressorConfigType::kHybrid));
}

TEST_CASE("REVPHSimTest SetPressureSwitchEnabled", "[wpilibc][simulation]") {
  PneumaticHub ph{CANPort::CAN_S0};
  REVPHSim sim(ph);
  sim.ResetData();

  BooleanCallback callback;
  auto cb = sim.RegisterPressureSwitchCallback(callback.GetCallback(), false);

  CHECK_FALSE(ph.GetPressureSwitch());

  sim.SetPressureSwitch(true);
  CHECK(sim.GetPressureSwitch());
  CHECK(ph.GetPressureSwitch());
  CHECK(callback.WasTriggered());
  CHECK(callback.GetLastValue());
}

TEST_CASE("REVPHSimTest SetCompressorCurrent", "[wpilibc][simulation]") {
  PneumaticHub ph{CANPort::CAN_S0};
  REVPHSim sim(ph);
  sim.ResetData();

  DoubleCallback callback;
  auto cb =
      sim.RegisterCompressorCurrentCallback(callback.GetCallback(), false);

  sim.SetCompressorCurrent(35.04);
  CHECK(35.04 == sim.GetCompressorCurrent());
  CHECK(35.04_A == ph.GetCompressorCurrent());
  CHECK(callback.WasTriggered());
  CHECK(35.04 == callback.GetLastValue());
}
}  // namespace prosdk::sim
