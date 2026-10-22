// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Upstream generates WPILibVersion.cpp from wpilibc/src/generate/WPILibVersion.cpp.in at build
// time (the Gradle version). prosdk reports its own version (keep in sync with PROSDK_VERSION in
// src/prosdk.h) and the pinned upstream commit (tools/upstream/wpilib.py).

#include "prosdk/system/WPILibVersion.hpp"

const char* GetWPILibVersion() {
  return "prosdk 2027.0.0-alpha.1 (WPILib b68effca)";
}
