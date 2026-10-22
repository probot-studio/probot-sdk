// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// prosdk_drivers_tests: the hand-written drivers (src/prosdk/drivers) against the HAL sim.

#include <catch2/catch_session.hpp>

#include "prosdk/hal/HAL.h"

int main(int argc, char** argv) {
  HAL_Initialize();
  return Catch::Session().run(argc, argv);
}
