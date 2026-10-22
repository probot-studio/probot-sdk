// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/SimDeviceSim.hpp"

#include <string_view>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "prosdk/hal/SimDevice.h"

using namespace prosdk::sim;

TEST_CASE("SimDeviceSimTest Basic", "[wpilibc][simulation]") {
  prosdk::hal::SimDevice dev{"test"};
  prosdk::hal::SimBoolean devBool =
      dev.CreateBoolean("bool", prosdk::hal::SimDevice::Direction::kInput, false);

  SimDeviceSim sim{"test"};
  prosdk::hal::SimBoolean simBool = sim.GetBoolean("bool");
  CHECK_FALSE(simBool.Get());
  simBool.Set(true);
  CHECK(devBool.Get());

  CHECK(sim.GetName() == "test");
}

TEST_CASE("SimDeviceSimTest EnumerateDevices", "[wpilibc][simulation]") {
  prosdk::hal::SimDevice dev{"test"};

  bool foundit = false;
  SimDeviceSim::EnumerateDevices(
      "te", [&](const char* name, HAL_SimDeviceHandle handle) {
        if (std::string_view(name) == "test") {
          foundit = true;
        }
      });
  CHECK(foundit);
}
