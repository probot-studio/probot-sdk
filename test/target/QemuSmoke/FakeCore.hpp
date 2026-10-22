// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// A fake probot-core for the QEMU smoke test: the CoreOps table (prosdk/platform/CoreOps.hpp)
// that the sketch-side bridge (prosdk/probot/CoreBridge.hpp) would fill from <probot.h>, installed
// at static initialization exactly as the bridge does it. probot-core itself needs WiFi, which
// QEMU does not emulate, so the test drives core's framework hooks (detail::Boot / PreCycle /
// PostCycle / EmergencyStop) itself (CaseRobot.cpp).

#pragma once

#include <stdint.h>

#include <atomic>
#include <string>

namespace qemu::fakecore {

struct State {
  std::atomic<uint8_t> phase{0};         // CorePhase
  std::atomic<uint8_t> selectedMode{1};  // CoreMode (teleop)
  std::atomic<bool> outputs{false};
  std::atomic<bool> estop{false};
  std::atomic<bool> attached{true};
  std::atomic<uint32_t> packetSeq{0};
  std::atomic<uint32_t> periodMs{20};
  std::atomic<uint32_t> periodRequests{0};
  std::atomic<uint32_t> lastPeriodRequest{0};
  std::atomic<uint32_t> overruns{0};
  // dashboard store
  std::atomic<uint32_t> topics{0};
  std::atomic<uint32_t> sets{0};
  std::atomic<uint32_t> schemas{0};
};

State& Get();

/** Everything written through CoreOps::consoleWrite (the Driver Station console). */
std::string Console();

/** True when the table was installed at static initialization. */
bool Installed();

}  // namespace qemu::fakecore
