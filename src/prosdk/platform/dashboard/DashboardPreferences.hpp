// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// prosdk::Preferences on the Driver Station page. Upstream keeps the preferences in the persistent
// NetworkTables table /Preferences, so dashboards show and edit them; ntcore is not ported and the
// values live in the platform store (prosdk/platform/PreferencesStore.hpp, NVS on the robot).
// TrackPreferences() mirrors that store as robust tunables:
//
//   /Preferences/<key>   one topic per preference, the store's type (boolean, integer, float,
//                        double, string), flags TUNABLE | MUTABLE | ROBUST
//
// - Keys added, changed or removed at run time (Preferences::Set*/Init*/Remove, from any thread)
//   are reflected at the next TunableRegistry::Update() (the robot loop), which runs the mirror
//   first (the registry's pre-update callback); nothing is published from other threads.
// - A Driver Station edit is applied by TunableRegistry::Update() like any tunable write and then
//   written to the store (Preferences' Set: persisted to NVS, rewritten only if the value
//   changed), so Preferences::Get* returns it and it survives a reboot.
// - The tunables under /Preferences go to their own dashboard backend
//   (prosdk::backend::ProbotTunableBackend with an empty store prefix, registered for
//   "/Preferences"), so the store path is /Preferences/<key> as in upstream, not
//   /Tunables/Preferences/<key>. A user tunable published under "/Preferences" lands there too.
//
// The robot host calls TrackPreferences() from RegisterDashboardBackends().

#pragma once

namespace prosdk::platform::dashboard {

/**
 * Registers the /Preferences tunable backend, installs the mirror as the TunableRegistry
 * pre-update callback and (re)publishes every preference at the next update. Call once (the robot
 * host does); calling it again starts the mirror over.
 */
void TrackPreferences();

/**
 * Brings the /Preferences tunables in line with the store (added, changed, removed keys, changed
 * types). Runs inside TunableRegistry::Update(); exposed for tests. No-op before
 * TrackPreferences().
 */
void SyncPreferences();

/** Host tests only: drops the mirror (its tunables are unregistered) and stops tracking. */
void ResetPreferencesForTest();

}  // namespace prosdk::platform::dashboard
