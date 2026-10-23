// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// The default ("") telemetry backend and DataLogManager's copy of it in the data log.
//
// Upstream RobotBase registers the NetworkTables telemetry backend for "" and DataLogManager logs
// every NetworkTables topic into its .wpilog under "NT:" (LogNetworkTables()), so every /Telemetry
// value ends up in the log. ntcore is not ported. Here the robot host sets the Driver Station
// backend as the default backend (RegisterDashboardBackends()), and DataLogManager (patch
// wpilibc/0016) adds a prosdk::backend::DataLogTelemetryBackend next to it while it logs
// "NetworkTables": the backend registered for "" is then a MultiTelemetryBackend of both, and each
// telemetry value is also written to the log as "NT:/Telemetry/<path>", the name upstream's log
// gives it (AdvantageScope finds the same keys).
//   - Switching the copy on or off keeps the default backend's entries: values logged before stay
//     on the Driver Station, also those logged only once or only on change. A value reaches the
//     log the next time it is logged (upstream's logger also records the current values at start).
//   - Without a default backend (host tests, a bare sketch) nothing is registered: telemetry is
//     not routed anywhere, as upstream without the NetworkTables backend. Clearing the default
//     backend (nullptr) after it was set registers a discard backend for "".
//   - A backend registered for "" by other code replaces this pair until the next call here; a
//     backend registered for a longer prefix gets those paths without the copy (the Driver
//     Station display does that, prosdk/platform/dashboard/Dashboard.cpp).

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
 * is started). nullptr: see the file comment. Thread-safe.
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

namespace detail {
/** Makes the backend registered for "" from the default backend and the log backend. */
using ComposeTelemetryBackends = std::shared_ptr<prosdk::telemetry::TelemetryBackend> (*)(
    std::shared_ptr<prosdk::telemetry::TelemetryBackend>,
    std::shared_ptr<prosdk::telemetry::TelemetryBackend>);

/**
 * StartTelemetryLog() without the backend types (TelemetryLogStart.cpp builds them), so a sketch
 * that never starts DataLogManager does not link them.
 */
void StartTelemetryLog(prosdk::log::DataLog& log,
                       std::shared_ptr<prosdk::telemetry::TelemetryBackend> logBackend,
                       ComposeTelemetryBackends compose);
}  // namespace detail

}  // namespace prosdk::platform
