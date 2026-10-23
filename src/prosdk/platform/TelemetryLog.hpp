// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// The default ("") telemetry backend and DataLogManager's copy of it in the data log.
//
// Upstream RobotBase registers the NetworkTables telemetry backend for "" and DataLogManager logs
// every NetworkTables topic into its .wpilog under "NT:" (LogNetworkTables(), on by default), so
// every /Telemetry value ends up in the log. ntcore is not ported. Here the robot host sets the
// Driver Station backend as the default backend (RegisterDashboardBackends()), and DataLogManager
// (patch wpilibc/0016) adds a prosdk::backend::DataLogTelemetryBackend next to it while it logs
// "NetworkTables": the backend registered for "" is then a MultiTelemetryBackend of both, and each
// telemetry value is also written to the log as "NT:/Telemetry/<path>", the name upstream's log
// gives it (AdvantageScope finds the same keys). Without a default backend (host tests, a bare
// sketch) nothing is registered: telemetry is not routed anywhere, as upstream without the
// NetworkTables backend. A backend registered for "" by other code replaces this pair until the
// next call here.

#pragma once

#include <memory>

namespace prosdk::log {
class DataLog;
}  // namespace prosdk::log

namespace prosdk::telemetry {
class TelemetryBackend;
}  // namespace prosdk::telemetry

namespace prosdk::platform {

/** Data log name prefix of the telemetry copy ("NT:" + the dashboard backend's "/Telemetry"). */
inline constexpr const char* kTelemetryLogPrefix = "NT:/Telemetry";

/**
 * Sets the default telemetry backend and registers it for "" (with the data log copy while one
 * is started). Thread-safe.
 */
void SetDefaultTelemetryBackend(std::shared_ptr<prosdk::telemetry::TelemetryBackend> backend);

/**
 * Starts copying telemetry into `log` under kTelemetryLogPrefix (replacing a copy into another
 * log). The log must outlive the copy: call StopTelemetryLog(log) before destroying it.
 * Thread-safe.
 */
void StartTelemetryLog(prosdk::log::DataLog& log);

/** Stops the copy into `log` (no-op if the copy goes to another log or none). Thread-safe. */
void StopTelemetryLog(prosdk::log::DataLog& log);

}  // namespace prosdk::platform
