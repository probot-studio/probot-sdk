// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/CTREPCMSim.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "callback_helpers/TestCallbackHelpers.hpp"
#include "prosdk/hardware/pneumatic/DoubleSolenoid.hpp"
#include "prosdk/hardware/pneumatic/PneumaticsControlModule.hpp"

namespace prosdk::sim {

TEST_CASE("CTREPCMSimTest InitializedCallback", "[wpilibc][simulation]") {
  CTREPCMSim sim;

  sim.ResetData();
  CHECK_FALSE(sim.GetInitialized());

  BooleanCallback callback;
  auto cb = sim.RegisterInitializedCallback(callback.GetCallback(), false);

  PneumaticsControlModule pcm{CANPort::CAN_S0};
  CHECK(sim.GetInitialized());
  CHECK(callback.WasTriggered());
  CHECK(callback.GetLastValue());
}

TEST_CASE("CTREPCMSimTest SolenoidOutput", "[wpilibc][simulation]") {
  PneumaticsControlModule pcm{CANPort::CAN_S0};
  CTREPCMSim sim(pcm);
  sim.ResetData();

  DoubleSolenoid doubleSolenoid{CANPort::CAN_S0,
                                prosdk::PneumaticsModuleType::CTRE_PCM, 3, 4};

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
  CHECK(0b00010000 == pcm.GetSolenoids());
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
  CHECK(0b00001000 == pcm.GetSolenoids());
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
  CHECK(0b00000000 == pcm.GetSolenoids());
  CHECK(0b00000000 == sim.GetAllSolenoidOutputs());
}

TEST_CASE("CTREPCMSimTest SetCompressorOn", "[wpilibc][simulation]") {
  PneumaticsControlModule pcm{CANPort::CAN_S0};
  CTREPCMSim sim(pcm);
  sim.ResetData();

  BooleanCallback callback;
  auto cb = sim.RegisterCompressorOnCallback(callback.GetCallback(), false);

  CHECK_FALSE(pcm.GetCompressor());
  CHECK_FALSE(pcm.GetCompressor());
  sim.SetCompressorOn(true);
  CHECK(sim.GetCompressorOn());
  CHECK(pcm.GetCompressor());
  CHECK(callback.WasTriggered());
  CHECK(callback.GetLastValue());
}

TEST_CASE("CTREPCMSimTest SetEnableDigital", "[wpilibc][simulation]") {
  PneumaticsControlModule pcm{CANPort::CAN_S0};
  CTREPCMSim sim(pcm);
  sim.ResetData();

  BooleanCallback callback;
  auto cb =
      sim.RegisterClosedLoopEnabledCallback(callback.GetCallback(), false);

  pcm.DisableCompressor();
  CHECK(pcm.GetCompressorConfigType() == CompressorConfigType::kDisabled);

  pcm.EnableCompressorDigital();
  CHECK(sim.GetClosedLoopEnabled());
  CHECK(pcm.GetCompressorConfigType() == CompressorConfigType::kDigital);
  CHECK(callback.WasTriggered());
  CHECK(callback.GetLastValue());
}

TEST_CASE("CTREPCMSimTest SetPressureSwitchEnabled", "[wpilibc][simulation]") {
  PneumaticsControlModule pcm{CANPort::CAN_S0};
  CTREPCMSim sim(pcm);
  sim.ResetData();

  BooleanCallback callback;
  auto cb = sim.RegisterPressureSwitchCallback(callback.GetCallback(), false);

  CHECK_FALSE(pcm.GetPressureSwitch());

  sim.SetPressureSwitch(true);
  CHECK(sim.GetPressureSwitch());
  CHECK(pcm.GetPressureSwitch());
  CHECK(callback.WasTriggered());
  CHECK(callback.GetLastValue());
}

TEST_CASE("CTREPCMSimTest SetCompressorCurrent", "[wpilibc][simulation]") {
  PneumaticsControlModule pcm{CANPort::CAN_S0};
  CTREPCMSim sim(pcm);
  sim.ResetData();

  DoubleCallback callback;
  auto cb =
      sim.RegisterCompressorCurrentCallback(callback.GetCallback(), false);

  sim.SetCompressorCurrent(35.04);
  CHECK(35.04 == sim.GetCompressorCurrent());
  CHECK(35.04_A == pcm.GetCompressorCurrent());
  CHECK(callback.WasTriggered());
  CHECK(35.04 == callback.GetLastValue());
}
}  // namespace prosdk::sim
