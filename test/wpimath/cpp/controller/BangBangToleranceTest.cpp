// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <memory>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/controller/BangBangController.hpp"
#include "prosdk/tunables/MockTunableBackend.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"
#include "prosdk/tunables/Tunables.hpp"

TEST_CASE("BangBangToleranceTest InTolerance", "[wpimath]") {
  prosdk::math::BangBangController controller{0.1};

  controller.SetSetpoint(1);
  controller.Calculate(1);
  CHECK(controller.AtSetpoint());
}

TEST_CASE("BangBangToleranceTest OutOfTolerance", "[wpimath]") {
  prosdk::math::BangBangController controller{0.1};

  controller.SetSetpoint(1);
  controller.Calculate(0);
  CHECK_FALSE(controller.AtSetpoint());
}

TEST_CASE("BangBangToleranceTest TunedMembersUpdateState", "[wpimath]") {
  prosdk::tunables::TunableRegistry::Reset();
  auto backend = std::make_shared<prosdk::tunables::MockTunableBackend>();
  prosdk::tunables::TunableRegistry::RegisterBackend("", backend);

  prosdk::math::BangBangController controller{0.1};
  prosdk::tunables::Publish("bangbang", controller);

  auto toleranceUid = backend->GetUid("/bangbang/tolerance");
  auto setpointUid = backend->GetUid("/bangbang/setpoint");
  REQUIRE(toleranceUid.has_value());
  REQUIRE(setpointUid.has_value());
  CHECK_FALSE(
      prosdk::tunables::TunableRegistry::GetTunable(*toleranceUid).IsChanged());

  backend->SetDouble("/bangbang/tolerance", 0.25);
  backend->SetDouble("/bangbang/setpoint", 4.0);
  prosdk::tunables::TunableRegistry::Update();

  CHECK(controller.GetTolerance() == 0.25);
  CHECK(controller.GetSetpoint() == 4.0);

  controller.SetTolerance(0.5);
  controller.SetSetpoint(6.0);

  CHECK(prosdk::tunables::TunableRegistry::GetTunable(*toleranceUid).IsChanged());
  CHECK(prosdk::tunables::TunableRegistry::GetTunable(*setpointUid).IsChanged());

  prosdk::tunables::TunableRegistry::Reset();
}
