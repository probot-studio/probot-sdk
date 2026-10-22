// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <string>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/hal/DriverStation.h"
#include "prosdk/hal/simulation/MockHooks.h"
#include "prosdk/util/string.hpp"

namespace {
std::string gDisplayAnsi;

int32_t CaptureDisplayAnsi(const struct WPI_String* data) {
  gDisplayAnsi = prosdk::util::to_string_view(data);
  return 0;
}
}  // namespace

TEST_CASE("DriverStationDisplayAnsiTest WriteDisplayAnsiUsesSimulationHook",
          "[hal]") {
  HALSIM_SetWriteDisplayAnsi(nullptr);
  gDisplayAnsi.clear();
  HALSIM_SetWriteDisplayAnsi(CaptureDisplayAnsi);

  auto data = prosdk::util::make_string("Robot display");
  HAL_WriteDisplayAnsi(&data);

  HALSIM_SetWriteDisplayAnsi(nullptr);
  CHECK("Robot display" == gDisplayAnsi);
}
