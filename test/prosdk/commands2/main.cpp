// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include <catch2/catch_session.hpp>

#include "prosdk/hal/HAL.h"

int main(int argc, char** argv) {
  HAL_Initialize();
  return Catch::Session().run(argc, argv);
}
