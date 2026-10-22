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
 *     backend for "", replacing what was registered for "" before;
 *   - installs the alert tracking backend: active alerts appear under /Telemetry/Alerts/<group>
 *     (.type "Alerts", string[] errors/warnings/infos, newest first);
 *   - adds a pre-cycle callback (Driver Station writes are drained into the tunable backend,
 *     topics that waited for the store are published) and a post-cycle callback (alert
 *     publication) to core's user task.
 * Tunable writes are applied by prosdk::tunables::TunableRegistry::Update(), which the robot
 * framework runs every loop (a bare sketch calls it itself).
 */
void RegisterDashboardBackends();

}  // namespace prosdk::platform
