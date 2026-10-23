// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Driver Station dashboard for prosdk telemetry, tunables and alerts, on probot-core's dashboard
// topic store (the counterpart of the NetworkTables backends RobotBase registers upstream).

#pragma once

namespace prosdk::platform {

/**
 * Connects telemetry, tunables and alerts to the Driver Station page. Idempotent and thread-safe;
 * the robot framework calls it once at startup (a sketch without the framework may call it from
 * setup()). It:
 *   - registers prosdk::backend::ProbotTelemetryBackend("/Telemetry") as the telemetry backend
 *     for "" (all paths) and prosdk::backend::ProbotTunableBackend("/Tunables") as the tunable
 *     backend for "", replacing what was registered for "" before (the telemetry backend
 *     through prosdk::platform::SetDefaultTelemetryBackend(): while DataLogManager logs
 *     "NetworkTables", telemetry is also copied into its log, prosdk/platform/TelemetryLog.hpp);
 *   - shows prosdk::Preferences as robust tunables /Preferences/<key> (own tunable backend for
 *     "/Preferences"); Driver Station edits are written back to the Preferences store
 *     (prosdk/platform/dashboard/DashboardPreferences.hpp);
 *   - installs the alert tracking backend: active alerts appear under /Telemetry/Alerts/<group>
 *     (.type "Alerts", string[] errors/warnings/infos, newest first);
 *   - adds a pre-cycle callback (Driver Station writes are drained into the tunable backend,
 *     topics that waited for the store are published) and a post-cycle callback (alert
 *     publication) to core's user task.
 * Tunable writes are applied by prosdk::tunables::TunableRegistry::Update(), which the robot
 * framework runs every loop (a bare sketch calls it itself).
 */
void RegisterDashboardBackends();

/**
 * Erases every tunable value probot-core keeps across reboots (tunables published with
 * TunableConfig::properties {"persistent": true}; core's probot::dashboard::clearPersisted()).
 * Core stores the Driver Station tunes the robot applied, and a stored value wins over the code
 * default after a reboot: call this once after changing a persistent tunable's default in code.
 * The tunables keep their current values; core stores the next Driver Station write again. A
 * persistent tunable published again afterwards (OpMode change) keeps its own value, not one
 * tuned before the clear, until it is tuned again. Writes flash (stalls both cores briefly): call
 * it while the robot is stopped. Returns false when core has no dashboard
 * persistence (probot-core R12, CoreOps ABI 6) or the erase failed.
 */
bool ClearPersistedTunables();

}  // namespace prosdk::platform
