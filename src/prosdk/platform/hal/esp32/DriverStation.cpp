// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// The MrcLibDs structure and cache semantics follow allwpilib hal/src/main/native/sim/
// DriverStation.cpp (WPILib BSD license).
//
// ESP32-S3 HAL backend: the Driver Station is probot-core's browser Driver Station
// (design/hal-esp32.md, "Driver Station"). A pre-cycle callback on core's user task (the "pump")
// reads core's state through CoreOps every cycle; when the Driver Station sent a new packet, the
// control word changed or a connected joystick was lost (core's input timeout) it stores a new
// snapshot and wakes the new-data events. HAL_RefreshDSData
// makes the latest snapshot the one the getters return, as on the other backends.
//
// Mapping:
//  - control word: robot mode from core's phase (AUTO_* autonomous, TELEOP_* teleop, UTILITY_*
//    utility, otherwise the mode selected on the Driver Station); enabled = core outputsEnabled()
//    (true only in a started RUN phase); e-stop latched; DS attached = a Driver Station client is
//    connected; never FMS attached.
//  - opmodes (probot-core R7): HAL_SetOpModeOptions publishes the options as core's opmode list
//    (core option id = WPILib opmode id; mode from the id's robot-mode bits). The control word
//    carries core's activeOpModeId() (latched at INIT) in the INIT and RUN phases and its
//    selectedOpModeId() while idle, so OpModeRobot constructs the selected opmode while disabled
//    and starts it on enable, as with the FRC Driver Station.
//  - joysticks (probot-core R8): core gamepad port n (0-5) is HAL joystick n, read with
//    ds::rawGamepad(n) (data only for a port that received a frame). Raw browser (W3C) buttons
//    and axes are remapped through core's active mapping profile into WPILib's Gamepad order
//    (SDL): buttons FACE_DOWN..DPAD_RIGHT (0..14), axes LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y,
//    LEFT_TRIGGER, RIGHT_TRIGGER (Y axes down-positive; triggers are core's analog values after
//    its deadzone, raw = the analog button value), POV 0 from core's 8-way D-pad; the name and
//    the standard-mapping flag go to the descriptor (prosdk/platform/hal/CoreJoystick.hpp).
//    Rumble and LEDs are not supported (ignored).
//  - no alliance station, match info or game data (zero/unknown); match time from core.
//  - DriverStationDisplay (HAL_WriteDisplayAnsi): to the console, ANSI removed, repeats dropped.

#include <atomic>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <format>
#include <mutex>
#include <string>
#include <type_traits>
#include <vector>

#include "Esp32Internal.hpp"
#include "prosdk/hal/DriverStationTypes.h"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/cpp/MrcLibDs.hpp"
#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/hal/CoreJoystick.hpp"
#include "prosdk/util/EventVector.hpp"
#include "prosdk/util/string.hpp"

using namespace prosdk::hal;
using prosdk::platform::CoreOps;
using prosdk::platform::GetCoreOps;

namespace {

struct Snapshot {
  Snapshot() { std::memset(this, 0, sizeof(*this)); }

  HAL_JoystickAxes axes[HAL_MAX_JOYSTICKS];
  HAL_JoystickPOVs povs[HAL_MAX_JOYSTICKS];
  HAL_JoystickButtons buttons[HAL_MAX_JOYSTICKS];
  HAL_JoystickTouchpads touchpads[HAL_MAX_JOYSTICKS];
  HAL_JoystickDescriptor descriptors[HAL_MAX_JOYSTICKS];
  double matchTime;
  HAL_ControlWord controlWord;
};
static_assert(std::is_standard_layout_v<Snapshot>);
static_assert(HAL_MAX_JOYSTICKS == prosdk::platform::kCoreMaxGamepads);

std::atomic_bool gShutdown{false};

HAL_RobotMode RobotModeFromCoreMode(uint8_t mode) {
  switch (mode) {
    case prosdk::platform::kCoreModeAuto:
      return HAL_ROBOT_MODE_AUTONOMOUS;
    case prosdk::platform::kCoreModeUtility:
      return HAL_ROBOT_MODE_UTILITY;
    default:
      return HAL_ROBOT_MODE_TELEOPERATED;
  }
}

HAL_RobotMode RobotModeFromCore(const CoreOps& ops, uint8_t phase) {
  switch (phase) {
    case prosdk::platform::kCorePhaseAutoInit:
    case prosdk::platform::kCorePhaseAutoRun:
      return HAL_ROBOT_MODE_AUTONOMOUS;
    case prosdk::platform::kCorePhaseTeleopInit:
    case prosdk::platform::kCorePhaseTeleopRun:
      return HAL_ROBOT_MODE_TELEOPERATED;
    case prosdk::platform::kCorePhaseUtilityInit:
    case prosdk::platform::kCorePhaseUtilityRun:
      return HAL_ROBOT_MODE_UTILITY;
    default:
      return RobotModeFromCoreMode(ops.selectedMode());
  }
}

// WPILib opmode robot mode -> core mode; false for HAL_ROBOT_MODE_UNKNOWN.
bool CoreModeFromRobotMode(HAL_RobotMode mode, uint8_t* out) {
  switch (mode) {
    case HAL_ROBOT_MODE_AUTONOMOUS:
      *out = prosdk::platform::kCoreModeAuto;
      return true;
    case HAL_ROBOT_MODE_TELEOPERATED:
      *out = prosdk::platform::kCoreModeTeleop;
      return true;
    case HAL_ROBOT_MODE_UTILITY:
      *out = prosdk::platform::kCoreModeUtility;
      return true;
    default:
      return false;
  }
}

class ProbotCoreDs final : public MrcLibDs {
 public:
  ProbotCoreDs();
  ~ProbotCoreDs() override { gShutdown = true; }

  int32_t sendError(bool isError, int32_t errorCode, const struct WPI_String* details,
                    const struct WPI_String* location, const struct WPI_String* callStack,
                    bool printMsg) override;
  int32_t sendConsoleLine(const struct WPI_String* line) override;
  int32_t sendProgramCrash(const struct WPI_String* details, const struct WPI_String* location,
                           const struct WPI_String* callStack) override;
  int32_t getControlWord(HAL_ControlWord* controlWord) override;
  int32_t getUncachedControlWord(HAL_ControlWord* controlWord) override;
  int32_t setOpModeOptions(const struct HAL_OpModeOption* options, int32_t count) override;
  int32_t getAllianceStation(HAL_AllianceStationID* allianceStation) override;
  int32_t getJoystickAxes(int32_t joystickNum, HAL_JoystickAxes* axes) override;
  int32_t getJoystickPOVs(int32_t joystickNum, HAL_JoystickPOVs* povs) override;
  int32_t getJoystickButtons(int32_t joystickNum, HAL_JoystickButtons* buttons) override;
  int32_t getJoystickTouchpads(int32_t joystickNum, HAL_JoystickTouchpads* touchpads) override;
  int32_t getAllJoystickData(int32_t joystickNum, HAL_JoystickAxes* axes, HAL_JoystickPOVs* povs,
                             HAL_JoystickButtons* buttons,
                             HAL_JoystickTouchpads* touchpads) override;
  int32_t getJoystickDescriptor(int32_t joystickNum, HAL_JoystickDescriptor* desc) override;
  int32_t getGameData(HAL_GameData* gameData) override;
  int32_t setJoystickRumble(int32_t joystickNum, int32_t leftRumble, int32_t rightRumble,
                            int32_t leftTriggerRumble, int32_t rightTriggerRumble) override;
  int32_t setJoystickLeds(int32_t joystickNum, int32_t leds) override;
  int32_t getMatchTime(double* matchTime) override;
  int32_t getMatchInfo(HAL_MatchInfo* info) override;
  int32_t getOutputsEnabled(bool* outputsEnabled) override;
  int32_t refreshDSData(bool* wasRefreshed) override;
  void provideNewDataEventHandle(WPI_EventHandle handle) override;
  void removeNewDataEventHandle(WPI_EventHandle handle) override;
  int32_t observeUserProgramStarting() override;
  int32_t observeUserProgram(HAL_ControlWord controlWord) override;
  int32_t getSystemTimeValid(bool* systemTimeValid) override;
  int32_t writeDisplayAnsi(const struct WPI_String* line) override;

  /** Pump: runs on core's user task before each loop iteration. */
  void Poll();

 private:
  HAL_ControlWord MakeControlWord(const CoreOps& ops);

  prosdk::util::EventVector m_newDataEvents;
  BackendPrintFunction m_backendPrintFunc;

  std::mutex m_cacheMutex;
  Snapshot m_latest;   // written by Poll
  Snapshot m_current;  // what the getters return (after HAL_RefreshDSData)
  bool m_hasNew = false;
  HAL_ControlWord m_newestControlWord;

  // Poll state (core's user task only)
  Snapshot m_scratch;  // not on the stack: about 2.5 KB
  prosdk::platform::CoreJoystickData m_stick;
  uint32_t m_lastSeq = 0;
  uint64_t m_lastWord = ~uint64_t{0};
  uint8_t m_connected = 0;  // ports (bit n = port n) connected in m_latest

  // DriverStationDisplay frames (ANSI stripped), last one written to the console
  std::mutex m_displayMutex;
  std::string m_lastDisplay;
};

ProbotCoreDs::ProbotCoreDs() {
  m_newestControlWord.value = 0;
  m_backendPrintFunc = [](bool, int32_t, const struct WPI_String*, const struct WPI_String*,
                          const struct WPI_String*, bool* forcePrintMsg) {
    // the console is the Driver Station console: always print
    *forcePrintMsg = true;
    return 0;
  };
}

HAL_ControlWord ProbotCoreDs::MakeControlWord(const CoreOps& ops) {
  const bool attached = ops.isAttached();
  if (!attached) {
    HAL_ControlWord controlWord;
    controlWord.value = 0;
    return controlWord;
  }
  const uint8_t phase = ops.phase();
  const HAL_RobotMode mode = RobotModeFromCore(ops, phase);
  // INIT/RUN: the opmode the run was INITed with; idle: the Driver Station's current choice
  const bool active =
      prosdk::platform::IsCoreInitPhase(phase) || prosdk::platform::IsCoreRunPhase(phase);
  const int64_t id =
      static_cast<int64_t>(active ? ops.activeOpModeId() : ops.selectedOpModeId());
  // core ids are WPILib ids (HAL_SetOpModeOptions); one of another mode is never reported
  const int64_t hash = id != 0 && HAL_OpMode_GetRobotMode(id) == mode ? HAL_OpMode_GetHash(id) : 0;
  return HAL_MakeControlWord(hash, mode, ops.outputsEnabled(), ops.isEStopped(), false, true);
}

void ProbotCoreDs::Poll() {
  if (gShutdown) {
    return;
  }
  const CoreOps& ops = GetCoreOps();
  const uint32_t seq = ops.packetSeq();
  const HAL_ControlWord controlWord = MakeControlWord(ops);
  // Without a new packet a joystick can still be lost: core neutralizes a port whose frames
  // stopped (input timeout) without counting a packet, so connected ports are re-read every cycle
  // and the last stick values of a lost gamepad are never kept.
  if (seq == m_lastSeq && controlWord.value == m_lastWord &&
      prosdk::platform::LostCoreJoysticks(ops, m_connected, &m_stick) == 0) {
    return;
  }
  m_lastSeq = seq;
  m_lastWord = controlWord.value;

  Snapshot& next = m_scratch;
  std::memset(&next, 0, sizeof(next));
  next.controlWord = controlWord;
  next.matchTime = ops.matchTimeSeconds();
  uint8_t connected = 0;
  for (int i = 0; i < HAL_MAX_JOYSTICKS; ++i) {
    // ports with no frame stay empty (disconnected)
    if (prosdk::platform::ReadCoreJoystick(ops, i, &m_stick)) {
      next.axes[i] = m_stick.axes;
      next.povs[i] = m_stick.povs;
      next.buttons[i] = m_stick.buttons;
      next.descriptors[i] = m_stick.descriptor;
      if (prosdk::platform::CoreJoystickConnected(m_stick)) {
        connected |= static_cast<uint8_t>(1u << i);
      }
    }
  }
  m_connected = connected;
  {
    std::scoped_lock lock(m_cacheMutex);
    m_latest = next;
    m_hasNew = true;
  }
  m_newDataEvents.Wakeup();
}

int32_t ProbotCoreDs::sendError(bool isError, int32_t errorCode, const struct WPI_String* details,
                                const struct WPI_String* location,
                                const struct WPI_String* callStack, bool printMsg) {
  return DefaultSendErrorImpl(isError, errorCode, details, location, callStack, printMsg,
                              m_backendPrintFunc);
}

int32_t ProbotCoreDs::sendConsoleLine(const struct WPI_String* line) {
  std::string text{prosdk::util::to_string_view(line)};
  text += '\n';
  prosdk::platform::ConsoleWrite(text);
  return 0;
}

int32_t ProbotCoreDs::sendProgramCrash(const struct WPI_String* details,
                                       const struct WPI_String* location,
                                       const struct WPI_String* callStack) {
  prosdk::platform::ConsoleWrite(std::format("Program Crash: {}\nLocation: {}\nCall Stack:\n{}\n",
                                             prosdk::util::to_string_view(details),
                                             prosdk::util::to_string_view(location),
                                             prosdk::util::to_string_view(callStack)));
  return 0;
}

int32_t ProbotCoreDs::getControlWord(HAL_ControlWord* controlWord) {
  if (gShutdown) {
    controlWord->value = 0;
    return HAL_INCOMPATIBLE_STATE;
  }
  std::scoped_lock lock(m_cacheMutex);
  *controlWord = m_newestControlWord;
  return 0;
}

int32_t ProbotCoreDs::getUncachedControlWord(HAL_ControlWord* controlWord) {
  if (gShutdown) {
    controlWord->value = 0;
    return HAL_INCOMPATIBLE_STATE;
  }
  *controlWord = MakeControlWord(GetCoreOps());
  return 0;
}

int32_t ProbotCoreDs::setOpModeOptions(const struct HAL_OpModeOption* options, int32_t count) {
  if (gShutdown) {
    return 0;
  }
  if (count < 0 || count > 1000 || (count != 0 && !options)) {
    return HAL_PARAMETER_OUT_OF_RANGE;
  }
  // core option id = WPILib opmode id (robot mode in bits 56-57, name hash below); textColor
  // and backgroundColor have no place in core's list and are dropped
  struct Strings {
    std::string name, group, description;
  };
  std::vector<Strings> strings;
  std::vector<prosdk::platform::CoreOpModeOption> list;
  strings.reserve(count);
  list.reserve(count);
  for (int32_t i = 0; i < count; ++i) {
    const HAL_OpModeOption& o = options[i];
    auto text = [](const WPI_String& s) {
      return s.str != nullptr ? std::string{s.str, s.len} : std::string{};
    };
    Strings& st = strings.emplace_back(Strings{text(o.name), text(o.group), text(o.description)});
    uint8_t mode = 0;
    if (!CoreModeFromRobotMode(HAL_OpMode_GetRobotMode(o.id), &mode) ||
        HAL_OpMode_GetHash(o.id) == 0) {
      esp32::Log(std::format("Driver Station: opmode \"{}\" has no robot mode or name hash "
                             "(id {:#x}); not listed",
                             st.name, o.id));
      continue;
    }
    list.push_back(prosdk::platform::CoreOpModeOption{
        static_cast<uint64_t>(o.id), mode, st.name.c_str(), st.group.c_str(),
        st.description.c_str()});
  }
  // validates, truncates to core's limits (with console warnings) and publishes
  if (!prosdk::platform::SetOpModeOptions(list.data(), list.size())) {
    return HAL_NO_AVAILABLE_RESOURCES;
  }
  return 0;
}

int32_t ProbotCoreDs::getAllianceStation(HAL_AllianceStationID* allianceStation) {
  *allianceStation = HAL_ALLIANCE_STATION_UNKNOWN;
  return gShutdown ? HAL_INCOMPATIBLE_STATE : 0;
}

#define CHECK_JOYSTICK_NUMBER(stickNum)                  \
  if ((stickNum) < 0 || (stickNum) >= HAL_MAX_JOYSTICKS) \
    return HAL_PARAMETER_OUT_OF_RANGE;

int32_t ProbotCoreDs::getJoystickAxes(int32_t joystickNum, HAL_JoystickAxes* axes) {
  if (gShutdown) {
    return HAL_INCOMPATIBLE_STATE;
  }
  CHECK_JOYSTICK_NUMBER(joystickNum);
  std::scoped_lock lock(m_cacheMutex);
  *axes = m_current.axes[joystickNum];
  return 0;
}

int32_t ProbotCoreDs::getJoystickPOVs(int32_t joystickNum, HAL_JoystickPOVs* povs) {
  if (gShutdown) {
    return HAL_INCOMPATIBLE_STATE;
  }
  CHECK_JOYSTICK_NUMBER(joystickNum);
  std::scoped_lock lock(m_cacheMutex);
  *povs = m_current.povs[joystickNum];
  return 0;
}

int32_t ProbotCoreDs::getJoystickButtons(int32_t joystickNum, HAL_JoystickButtons* buttons) {
  if (gShutdown) {
    return HAL_INCOMPATIBLE_STATE;
  }
  CHECK_JOYSTICK_NUMBER(joystickNum);
  std::scoped_lock lock(m_cacheMutex);
  *buttons = m_current.buttons[joystickNum];
  return 0;
}

int32_t ProbotCoreDs::getJoystickTouchpads(int32_t joystickNum,
                                           HAL_JoystickTouchpads* touchpads) {
  if (gShutdown) {
    return HAL_INCOMPATIBLE_STATE;
  }
  CHECK_JOYSTICK_NUMBER(joystickNum);
  std::scoped_lock lock(m_cacheMutex);
  *touchpads = m_current.touchpads[joystickNum];
  return 0;
}

int32_t ProbotCoreDs::getAllJoystickData(int32_t joystickNum, HAL_JoystickAxes* axes,
                                         HAL_JoystickPOVs* povs, HAL_JoystickButtons* buttons,
                                         HAL_JoystickTouchpads* touchpads) {
  if (gShutdown) {
    return HAL_INCOMPATIBLE_STATE;
  }
  CHECK_JOYSTICK_NUMBER(joystickNum);
  std::scoped_lock lock(m_cacheMutex);
  *axes = m_current.axes[joystickNum];
  *povs = m_current.povs[joystickNum];
  *buttons = m_current.buttons[joystickNum];
  *touchpads = m_current.touchpads[joystickNum];
  return 0;
}

int32_t ProbotCoreDs::getJoystickDescriptor(int32_t joystickNum, HAL_JoystickDescriptor* desc) {
  CHECK_JOYSTICK_NUMBER(joystickNum);
  std::scoped_lock lock(m_cacheMutex);
  *desc = m_current.descriptors[joystickNum];
  return 0;
}

int32_t ProbotCoreDs::getGameData(HAL_GameData* gameData) {
  std::memset(gameData, 0, sizeof(*gameData));
  return gShutdown ? HAL_INCOMPATIBLE_STATE : 0;
}

int32_t ProbotCoreDs::setJoystickRumble(int32_t, int32_t, int32_t, int32_t, int32_t) {
  return 0;  // the browser Driver Station has no rumble output
}

int32_t ProbotCoreDs::setJoystickLeds(int32_t, int32_t) {
  return 0;
}

int32_t ProbotCoreDs::getMatchTime(double* matchTime) {
  if (gShutdown) {
    *matchTime = 0;
    return HAL_INCOMPATIBLE_STATE;
  }
  std::scoped_lock lock(m_cacheMutex);
  *matchTime = m_current.matchTime;
  return 0;
}

int32_t ProbotCoreDs::getMatchInfo(HAL_MatchInfo* info) {
  std::memset(info, 0, sizeof(*info));
  info->matchType = HAL_MATCH_TYPE_NONE;
  return 0;
}

int32_t ProbotCoreDs::getOutputsEnabled(bool* outputsEnabled) {
  if (gShutdown) {
    *outputsEnabled = false;
    return HAL_INCOMPATIBLE_STATE;
  }
  std::scoped_lock lock(m_cacheMutex);
  *outputsEnabled = HAL_ControlWord_IsEnabled(m_newestControlWord) &&
                    HAL_ControlWord_IsDSAttached(m_newestControlWord);
  return 0;
}

int32_t ProbotCoreDs::refreshDSData(bool* wasRefreshed) {
  if (gShutdown) {
    *wasRefreshed = false;
    return HAL_INCOMPATIBLE_STATE;
  }
  const bool attached = GetCoreOps().isAttached();
  std::scoped_lock lock(m_cacheMutex);
  *wasRefreshed = m_hasNew;
  if (m_hasNew) {
    m_current = m_latest;
    m_hasNew = false;
  }
  if (!attached) {
    // as on the other backends: no Driver Station means a zero control word
    m_current.controlWord.value = 0;
  }
  m_newestControlWord = m_current.controlWord;
  return 0;
}

void ProbotCoreDs::provideNewDataEventHandle(WPI_EventHandle handle) {
  if (gShutdown) {
    return;
  }
  prosdk::hal::init::CheckInit();
  m_newDataEvents.Add(handle);
}

void ProbotCoreDs::removeNewDataEventHandle(WPI_EventHandle handle) {
  if (gShutdown) {
    return;
  }
  m_newDataEvents.Remove(handle);
}

int32_t ProbotCoreDs::observeUserProgramStarting() {
  return 0;  // core reports the program state on the Driver Station itself
}

int32_t ProbotCoreDs::observeUserProgram(HAL_ControlWord) {
  return 0;
}

// "System time" is the wall clock (std::chrono::system_clock, DataLogManager file names and its
// systemTime entry), not the HAL's monotonic clock. The ESP32's wall clock starts at 1970 after
// boot and nothing sets it yet (core-requests.md R10: set it from the browser), so it is valid
// only once something (the sketch's SNTP or settimeofday) moved it past 2024-01-01.
int32_t ProbotCoreDs::getSystemTimeValid(bool* systemTimeValid) {
  constexpr std::time_t k20240101 = 1704067200;
  *systemTimeValid = std::time(nullptr) >= k20240101;
  return 0;
}

// The Driver Station page has no display panel: DriverStationDisplay text goes to the console with
// the ANSI escape sequences removed. DriverStationDisplay::UpdateLines() resends the whole frame
// every 230 ms; a frame equal to the previous one (and an empty one: a clear) is not repeated.
int32_t ProbotCoreDs::writeDisplayAnsi(const struct WPI_String* line) {
  if (line == nullptr || line->str == nullptr || line->len == 0) {
    return 0;
  }
  std::string text;
  text.reserve(line->len);
  for (size_t i = 0; i < line->len; ++i) {
    const char c = line->str[i];
    if (c == '\033') {
      // CSI: ESC [ parameters/intermediates, then one final byte in 0x40..0x7E
      if (i + 1 < line->len && line->str[i + 1] == '[') {
        i += 2;
        while (i < line->len && !(line->str[i] >= 0x40 && line->str[i] <= 0x7E)) {
          ++i;
        }
      } else {
        ++i;  // two-byte escape
      }
      continue;
    }
    text.push_back(c);
  }
  if (text.find_first_not_of(" \t\r\n") == std::string::npos) {
    return 0;
  }
  if (text.back() != '\n') {
    text.push_back('\n');
  }
  {
    std::scoped_lock lock(m_displayMutex);
    if (text == m_lastDisplay) {
      return 0;
    }
    m_lastDisplay = text;
  }
  prosdk::platform::ConsoleWrite(text);
  return 0;
}

ProbotCoreDs& Instance() {
  static ProbotCoreDs impl;
  return impl;
}

void PumpCallback(void*) {
  Instance().Poll();
}

}  // namespace

namespace prosdk::hal {

MrcLibDs* GetDefaultDriverStationImpl() {
  return &Instance();
}

namespace init {
void InitializeDriverStationPump() {
  Instance();
  if (!prosdk::platform::AddCycleCallback(prosdk::platform::CyclePhase::kPre, PumpCallback,
                                          nullptr)) {
    esp32::Log("Driver Station: no free core cycle callback; joystick data will not update");
  }
}
}  // namespace init

}  // namespace prosdk::hal
