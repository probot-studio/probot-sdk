// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// prosdk umbrella header (library.properties includes=prosdk.h). Later waves add the robot
// framework, commands, math and hardware headers here.

#pragma once

#define PROSDK_VERSION "2027.0.0-alpha.1"

#include "prosdk/platform/config.hpp"
#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/tunables/Tunables.hpp"
#include "prosdk/util/Alert.hpp"
