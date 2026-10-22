// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Alerts on the Driver Station page: the replacement of the SystemCore alert transport
// (MrcLibAlert, not ported). TrackAlerts() installs a WPI_AlertBackend that delegates to the
// backend installed before it (the wpiutil default keeps the alerts) and notes every change;
// PublishAlerts() (core's post-cycle callback, see RegisterDashboardBackends()) then logs each
// alert group through telemetry as the WPILib "Alerts" widget layout:
//
//   /Telemetry/Alerts/<group>/.type     "Alerts"
//   /Telemetry/Alerts/<group>/errors    string[]  active kHigh alerts, newest first
//   /Telemetry/Alerts/<group>/warnings  string[]  active kMedium alerts, newest first
//   /Telemetry/Alerts/<group>/infos     string[]  active kLow alerts, newest first
//
// (the default group is "Alerts", so prosdk::util::Alert{"id", "text", kHigh} shows up under
// /Telemetry/Alerts/Alerts). A group stays published with empty arrays once its alerts are gone.
//
// Alert objects may be created, set and destroyed from any task; publication happens on the task
// that calls PublishAlerts().

#pragma once

namespace prosdk::platform::dashboard {

/** Installs the change-tracking alert backend (idempotent; thread-safe). */
void TrackAlerts() noexcept;

/** Publishes the alert groups if an alert changed since the last call (one task at a time). */
void PublishAlerts() noexcept;

/** Host tests only: forgets the published groups and marks the alerts changed. */
void ResetAlertsForTest() noexcept;

}  // namespace prosdk::platform::dashboard
