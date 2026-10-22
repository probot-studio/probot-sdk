// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/BangBangController.hpp"

#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/tunables/TunableTable.hpp"

using namespace prosdk::math;

void BangBangController::LogTo(prosdk::telemetry::TelemetryTable& table) const {
  table.Log("tolerance", GetTolerance());
  table.Log("setpoint", GetSetpoint());
  table.Log("measurement", GetMeasurement());
  table.Log("error", GetError());
  table.Log("atSetpoint", AtSetpoint());
}

std::string_view BangBangController::GetTelemetryType() const {
  return "BangBangController";
}

void BangBangController::PublishTunable(prosdk::tunables::TunableTable& table) {
  table.Publish("tolerance", m_tolerance);
  table.Publish("setpoint", m_setpoint);
}

std::string_view BangBangController::GetTunableType() const {
  return "BangBangController";
}
