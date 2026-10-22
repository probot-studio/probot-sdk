// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <array>
#include <cstdio>
#include <string>
#include <string_view>

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "prosdk/driverstation/Joystick.hpp"
#include "prosdk/driverstation/internal/DriverStationBackend.hpp"
#include "prosdk/hal/simulation/DriverStationData.h"
#include "prosdk/simulation/AlertSim.hpp"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/simulation/SimHooks.hpp"
#include "prosdk/util/Alert.hpp"

namespace {

class StderrCapture {
 public:
  StderrCapture() {
    std::fflush(stderr);
    m_file = std::tmpfile();
    REQUIRE(m_file != nullptr);
    m_originalStderr = Dup(Fileno(stderr));
    REQUIRE(m_originalStderr >= 0);
    REQUIRE(Dup2(Fileno(m_file), Fileno(stderr)) >= 0);
  }

  StderrCapture(const StderrCapture&) = delete;
  StderrCapture& operator=(const StderrCapture&) = delete;

  ~StderrCapture() {
    if (m_originalStderr >= 0) {
      std::fflush(stderr);
      Dup2(m_originalStderr, Fileno(stderr));
      Close(m_originalStderr);
    }
    if (m_file) {
      std::fclose(m_file);
    }
  }

  std::string Stop() {
    std::fflush(stderr);
    REQUIRE(Dup2(m_originalStderr, Fileno(stderr)) >= 0);
    Close(m_originalStderr);
    m_originalStderr = -1;

    std::rewind(m_file);
    std::string output;
    std::array<char, 1024> buffer;
    while (auto count = std::fread(buffer.data(), 1, buffer.size(), m_file)) {
      output.append(buffer.data(), count);
    }
    return output;
  }

 private:
  static int Fileno(FILE* file) {
#ifdef _WIN32
    return _fileno(file);
#else
    return fileno(file);
#endif
  }

  static int Dup(int fd) {
#ifdef _WIN32
    return _dup(fd);
#else
    return dup(fd);
#endif
  }

  static int Dup2(int source, int dest) {
#ifdef _WIN32
    return _dup2(source, dest);
#else
    return dup2(source, dest);
#endif
  }

  static void Close(int fd) {
#ifdef _WIN32
    _close(fd);
#else
    close(fd);
#endif
  }

  FILE* m_file = nullptr;
  int m_originalStderr = -1;
};

static void ResetJoystickAlerts() {
  prosdk::sim::DriverStationSim::ResetData();
  HALSIM_SetJoystickButtonsAvailable(0, 0);
  HALSIM_SetJoystickAxesAvailable(0, 0);
  HALSIM_SetJoystickPOVsAvailable(0, 0);
  prosdk::sim::DriverStationSim::SetFmsAttached(false);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::internal::DriverStationBackend::SilenceJoystickConnectionAlert(true);
  prosdk::sim::AlertSim::ResetData();
  prosdk::internal::DriverStationBackend::SilenceJoystickConnectionAlert(false);
}

static bool IsDriverStationAlertActive(std::string_view id,
                                       std::string_view text,
                                       prosdk::util::Alert::Level level) {
  for (const auto& alert : prosdk::sim::AlertSim::GetActive()) {
    if (alert.group == "DriverStation" && alert.id == id &&
        alert.text == text && alert.level == level) {
      return true;
    }
  }

  return false;
}

static bool IsJoystickDisconnectedAlertActive(int stick) {
  std::string expectedText =
      "Joystick on port " + std::to_string(stick) +
      " not available, check if all controllers are plugged in";

  return IsDriverStationAlertActive(
      "joystick" + std::to_string(stick) + "Disconnected", expectedText,
      prosdk::util::Alert::Level::kHigh);
}

}  // namespace

TEST_CASE("DriverStation joystick connected", "[wpilibc][driverstation]") {
  prosdk::sim::DriverStationSim::ResetData();
  prosdk::sim::DriverStationSim::NotifyNewData();

  auto [axes, buttons, povs, expected] =
      GENERATE(table<int, int, int, bool>({{0, 0, 0, false},
                                           {1, 0, 0, true},
                                           {0, 1, 0, true},
                                           {0, 0, 1, true},
                                           {1, 1, 1, true},
                                           {4, 10, 1, true}}));

  prosdk::sim::DriverStationSim::SetJoystickAxesMaximumIndex(1, axes);
  prosdk::sim::DriverStationSim::SetJoystickButtonsMaximumIndex(1, buttons);
  prosdk::sim::DriverStationSim::SetJoystickPOVsMaximumIndex(1, povs);
  prosdk::sim::DriverStationSim::NotifyNewData();

  REQUIRE(expected ==
          prosdk::internal::DriverStationBackend::IsJoystickConnected(1));
}

TEST_CASE("DriverStation joystick connection alerts",
          "[wpilibc][driverstation]") {
  ResetJoystickAlerts();

  auto [fmsAttached, silenceAlerts, expectedSilenced, expectedAlert] =
      GENERATE(table<bool, bool, bool, bool>({{false, true, true, false},
                                              {false, false, false, true},
                                              {true, true, false, true},
                                              {true, false, false, true}}));

  StderrCapture capture;

  prosdk::sim::DriverStationSim::SetFmsAttached(fmsAttached);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::internal::DriverStationBackend::SilenceJoystickConnectionAlert(
      silenceAlerts);

  prosdk::Joystick joystick{0};
  joystick.GetHID().GetRawButton(1);

  prosdk::sim::StepTiming(1_s);
  CHECK(prosdk::internal::DriverStationBackend::
            IsJoystickConnectionAlertSilenced() == expectedSilenced);
  CHECK(IsJoystickDisconnectedAlertActive(0) == expectedAlert);
  CHECK(capture.Stop() == "");

  ResetJoystickAlerts();
}

TEST_CASE("DriverStation joystick resource alerts",
          "[wpilibc][driverstation]") {
  ResetJoystickAlerts();

  HALSIM_SetJoystickButtonsAvailable(0, 1);
  HALSIM_SetJoystickAxesAvailable(0, 1);
  HALSIM_SetJoystickPOVsAvailable(0, 1);
  uint8_t fingerCounts[HAL_MAX_JOYSTICK_TOUCHPADS] = {1, 0};
  HALSIM_SetJoystickTouchpadCounts(0, 1, fingerCounts);
  prosdk::sim::DriverStationSim::NotifyNewData();

  StderrCapture capture;

  prosdk::internal::DriverStationBackend::GetStickButton(0, 1);
  prosdk::internal::DriverStationBackend::GetStickButton(0, 2);
  prosdk::internal::DriverStationBackend::GetStickAxis(0, 1);
  prosdk::internal::DriverStationBackend::GetStickPOV(0, 1);
  prosdk::internal::DriverStationBackend::GetStickTouchpadFinger(0, 0, 1);

  CHECK(IsDriverStationAlertActive("joystick0ButtonUnavailable",
                                   "Joystick Button 2 on port 0 not available",
                                   prosdk::util::Alert::Level::kMedium));
  CHECK(IsDriverStationAlertActive("joystick0AxisUnavailable",
                                   "Joystick axis 1 on port 0 not available",
                                   prosdk::util::Alert::Level::kMedium));
  CHECK(IsDriverStationAlertActive("joystick0POVUnavailable",
                                   "Joystick POV 1 on port 0 not available",
                                   prosdk::util::Alert::Level::kMedium));
  CHECK(IsDriverStationAlertActive(
      "joystick0TouchpadFingerUnavailable",
      "Joystick touchpad finger 1 on touchpad 0 on port 0 not available",
      prosdk::util::Alert::Level::kMedium));
  CHECK_FALSE(IsJoystickDisconnectedAlertActive(0));
  CHECK(capture.Stop() == "");

  ResetJoystickAlerts();
}

TEST_CASE(
    "DriverStation joystick alert collision does not commit partial "
    "alerts",
    "[wpilibc][driverstation]") {
  ResetJoystickAlerts();

  HALSIM_SetJoystickButtonsAvailable(0, 1);
  HALSIM_SetJoystickAxesAvailable(0, 1);
  HALSIM_SetJoystickPOVsAvailable(0, 1);
  prosdk::sim::DriverStationSim::NotifyNewData();

  {
    prosdk::util::Alert collision{"DriverStation", "joystick0AxisUnavailable",
                               "collision", prosdk::util::Alert::Level::kMedium};
    REQUIRE(collision);

    StderrCapture capture;

    CHECK_FALSE(prosdk::internal::DriverStationBackend::GetStickButton(0, 2));
    CHECK(capture.Stop() == "");

    auto alerts = prosdk::sim::AlertSim::GetAll();
    REQUIRE(alerts.size() == 1u);
    CHECK(alerts[0].group == "DriverStation");
    CHECK(alerts[0].id == "joystick0AxisUnavailable");
    CHECK(alerts[0].text == "collision");
    CHECK(alerts[0].level == prosdk::util::Alert::Level::kMedium);
    CHECK_FALSE(alerts[0].isActive());

    CHECK(prosdk::internal::DriverStationBackend::GetStickAxis(0, 1) == 0.0);
    CHECK(prosdk::sim::AlertSim::GetAll().size() == 1u);
  }

  CHECK_FALSE(prosdk::internal::DriverStationBackend::GetStickButton(0, 2));
  CHECK(IsDriverStationAlertActive("joystick0ButtonUnavailable",
                                   "Joystick Button 2 on port 0 not available",
                                   prosdk::util::Alert::Level::kMedium));

  ResetJoystickAlerts();
}
