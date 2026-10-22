// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// The MrcLibDs structure and cache semantics follow allwpilib hal/src/main/native/sim/
// DriverStation.cpp (WPILib BSD license).
//
// ESP32-S3 HAL backend: the Driver Station is probot-core's browser Driver Station
// (design/hal-esp32.md, "Driver Station"). A pre-cycle callback on core's user task (the "pump")
// reads core's state through CoreOps every cycle; when the Driver Station sent a new packet or the
// control word changed it stores a new snapshot and wakes the new-data events. HAL_RefreshDSData
// makes the latest snapshot the one the getters return, as on the other backends.
//
// Mapping:
//  - control word: robot mode from core's phase (AUTO_* autonomous, TELEOP_* teleop, otherwise
//    the mode selected on the Driver Station); enabled = core outputsEnabled(); e-stop latched;
//    DS attached = a Driver Station client is connected; never FMS attached. The opmode is the
//    first option registered with HAL_SetOpModeOptions for that robot mode (core selects only
//    the robot mode).
//  - joysticks: core gamepad port n is HAL joystick n. Raw browser (W3C) buttons and axes are
//    remapped through core's active mapping profile into WPILib's Gamepad order (SDL): buttons
//    FACE_DOWN..DPAD_RIGHT (0..14), axes LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y, LEFT_TRIGGER,
//    RIGHT_TRIGGER (Y axes down-positive; triggers 0 or 1 since core only has trigger buttons),
//    one POV from the D-pad. Rumble and LEDs are not supported (ignored).
//  - no alliance station, match info or game data (zero/unknown); match time from core.

#include <atomic>
#include <cstdio>
#include <cstring>
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
#include "prosdk/util/EventVector.hpp"
#include "prosdk/util/string.hpp"

using namespace prosdk::hal;
using prosdk::platform::CoreGamepad;
using prosdk::platform::CoreOps;
using prosdk::platform::GetCoreOps;

namespace {

// WPILib Gamepad (SDL) indices
enum : int {
  kFaceDown = 0,
  kFaceRight = 1,
  kFaceLeft = 2,
  kFaceUp = 3,
  kBack = 4,
  kGuide = 5,
  kStart = 6,
  kLeftStick = 7,
  kRightStick = 8,
  kLeftBumper = 9,
  kRightBumper = 10,
  kDpadUp = 11,
  kDpadDown = 12,
  kDpadLeft = 13,
  kDpadRight = 14,
  kNumButtons = 15,
};
enum : int {
  kAxisLeftX = 0,
  kAxisLeftY = 1,
  kAxisRightX = 2,
  kAxisRightY = 3,
  kAxisLeftTrigger = 4,
  kAxisRightTrigger = 5,
  kNumAxes = 6,
};

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

std::atomic_bool gShutdown{false};

bool RawButton(const CoreGamepad& pad, int index) {
  return index >= 0 && index < pad.buttonCount && index < 32 && ((pad.buttons >> index) & 1u);
}

// Raw axis (W3C, -1..1); false when the mapping has no such axis.
bool RawAxis(const CoreGamepad& pad, int index, float* value) {
  if (index < 0 || index >= pad.axisCount) {
    return false;
  }
  *value = pad.axes[index];
  return true;
}

int16_t ToRaw(float v) {
  if (v > 1.0f) {
    v = 1.0f;
  } else if (v < -1.0f) {
    v = -1.0f;
  }
  return static_cast<int16_t>(v * 32767.0f);
}

void SetAxis(HAL_JoystickAxes* axes, int index, float v) {
  axes->axes[index] = v;
  axes->raw[index] = ToRaw(v);
  axes->available |= static_cast<uint16_t>(1u << index);
}

void SetButton(HAL_JoystickButtons* buttons, int index, bool available, bool pressed) {
  if (!available) {
    return;
  }
  buttons->available |= uint64_t{1} << index;
  if (pressed) {
    buttons->buttons |= uint64_t{1} << index;
  }
}

// Fills one joystick from a core gamepad snapshot (all outputs were zeroed by the caller).
void ConvertGamepad(const CoreGamepad& pad, HAL_JoystickAxes* axes, HAL_JoystickPOVs* povs,
                    HAL_JoystickButtons* buttons, HAL_JoystickDescriptor* desc) {
  const auto& m = pad.mapping;
  float v = 0;
  if (RawAxis(pad, m.leftX, &v)) {
    SetAxis(axes, kAxisLeftX, v);
  }
  if (RawAxis(pad, m.leftY, &v)) {
    // raw is W3C (up negative) when core inverts it; WPILib/SDL is also up negative
    SetAxis(axes, kAxisLeftY, m.invertLeftY ? v : -v);
  }
  if (RawAxis(pad, m.rightX, &v)) {
    SetAxis(axes, kAxisRightX, v);
  }
  if (RawAxis(pad, m.rightY, &v)) {
    SetAxis(axes, kAxisRightY, m.invertRightY ? v : -v);
  }
  auto has = [&](int index) { return index >= 0 && index < pad.buttonCount; };
  if (has(m.leftTrigger)) {
    SetAxis(axes, kAxisLeftTrigger, RawButton(pad, m.leftTrigger) ? 1.0f : 0.0f);
  }
  if (has(m.rightTrigger)) {
    SetAxis(axes, kAxisRightTrigger, RawButton(pad, m.rightTrigger) ? 1.0f : 0.0f);
  }

  SetButton(buttons, kFaceDown, has(m.a), RawButton(pad, m.a));
  SetButton(buttons, kFaceRight, has(m.b), RawButton(pad, m.b));
  SetButton(buttons, kFaceLeft, has(m.x), RawButton(pad, m.x));
  SetButton(buttons, kFaceUp, has(m.y), RawButton(pad, m.y));
  SetButton(buttons, kBack, has(m.back), RawButton(pad, m.back));
  SetButton(buttons, kGuide, has(m.options), RawButton(pad, m.options));
  SetButton(buttons, kStart, has(m.start), RawButton(pad, m.start));
  SetButton(buttons, kLeftStick, has(m.leftStick), RawButton(pad, m.leftStick));
  SetButton(buttons, kRightStick, has(m.rightStick), RawButton(pad, m.rightStick));
  SetButton(buttons, kLeftBumper, has(m.leftBumper), RawButton(pad, m.leftBumper));
  SetButton(buttons, kRightBumper, has(m.rightBumper), RawButton(pad, m.rightBumper));

  bool up = false, down = false, left = false, right = false, dpad = false;
  if (m.dpadFromAxis) {
    // hat axis: -1 up, then clockwise in steps of 2/7; > 1 means centered
    float hat = 0;
    if (RawAxis(pad, m.dpadAxis, &hat)) {
      dpad = true;
      if (hat >= -1.05f && hat <= 1.05f) {
        int step = static_cast<int>((hat + 1.0f) * 3.5f + 0.5f) % 8;  // 0 = up, 2 = right, ...
        up = step == 7 || step == 0 || step == 1;
        right = step >= 1 && step <= 3;
        down = step >= 3 && step <= 5;
        left = step >= 5 && step <= 7;
      }
    }
  } else if (has(m.dpadUp) || has(m.dpadDown) || has(m.dpadLeft) || has(m.dpadRight)) {
    dpad = true;
    up = RawButton(pad, m.dpadUp);
    down = RawButton(pad, m.dpadDown);
    left = RawButton(pad, m.dpadLeft);
    right = RawButton(pad, m.dpadRight);
  }
  if (dpad) {
    SetButton(buttons, kDpadUp, true, up);
    SetButton(buttons, kDpadDown, true, down);
    SetButton(buttons, kDpadLeft, true, left);
    SetButton(buttons, kDpadRight, true, right);
    uint8_t pov = (up ? HAL_JOYSTICK_POV_UP : 0) | (right ? HAL_JOYSTICK_POV_RIGHT : 0) |
                  (down ? HAL_JOYSTICK_POV_DOWN : 0) | (left ? HAL_JOYSTICK_POV_LEFT : 0);
    povs->povs[0] = static_cast<HAL_JoystickPOV>(pov);
    povs->available = 1;
  }

  desc->isGamepad = 1;
  desc->gamepadType = 1;  // GenericHID::HIDType::STANDARD
  desc->supportedOutputs = 0;
  std::snprintf(desc->name, sizeof(desc->name), "%s",
                m.name != nullptr ? m.name : "Probot gamepad");
}

HAL_RobotMode RobotModeFromCore(const CoreOps& ops) {
  switch (ops.phase()) {
    case prosdk::platform::kCorePhaseAutoInit:
    case prosdk::platform::kCorePhaseAutoRun:
      return HAL_ROBOT_MODE_AUTONOMOUS;
    case prosdk::platform::kCorePhaseTeleopInit:
    case prosdk::platform::kCorePhaseTeleopRun:
      return HAL_ROBOT_MODE_TELEOPERATED;
    default:
      return ops.selectedMode() == prosdk::platform::kCoreModeAuto
                 ? HAL_ROBOT_MODE_AUTONOMOUS
                 : HAL_ROBOT_MODE_TELEOPERATED;
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

  std::mutex m_optionsMutex;
  std::vector<int64_t> m_opModeIds;  // HAL_OpModeOption::id of the registered options

  std::mutex m_cacheMutex;
  Snapshot m_latest;   // written by Poll
  Snapshot m_current;  // what the getters return (after HAL_RefreshDSData)
  bool m_hasNew = false;
  HAL_ControlWord m_newestControlWord;

  // Poll state (core's user task only)
  Snapshot m_scratch;  // not on the stack: about 2.5 KB
  uint32_t m_lastSeq = 0;
  uint64_t m_lastWord = ~uint64_t{0};
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
    HAL_ControlWord word;
    word.value = 0;
    return word;
  }
  HAL_RobotMode mode = RobotModeFromCore(ops);
  int64_t hash = 0;
  {
    std::scoped_lock lock(m_optionsMutex);
    for (int64_t id : m_opModeIds) {
      if (HAL_OpMode_GetRobotMode(id) == mode) {
        hash = HAL_OpMode_GetHash(id);
        break;
      }
    }
  }
  return HAL_MakeControlWord(hash, mode, ops.outputsEnabled(), ops.isEStopped(), false, true);
}

void ProbotCoreDs::Poll() {
  if (gShutdown) {
    return;
  }
  const CoreOps& ops = GetCoreOps();
  const uint32_t seq = ops.packetSeq();
  const HAL_ControlWord word = MakeControlWord(ops);
  if (seq == m_lastSeq && word.value == m_lastWord) {
    return;
  }
  m_lastSeq = seq;
  m_lastWord = word.value;

  Snapshot& next = m_scratch;
  std::memset(&next, 0, sizeof(next));
  next.controlWord = word;
  next.matchTime = ops.matchTimeSeconds();
  for (int i = 0; i < HAL_MAX_JOYSTICKS; ++i) {
    CoreGamepad pad;
    std::memset(&pad, 0, sizeof(pad));
    if (ops.rawGamepad(i, &pad)) {
      ConvertGamepad(pad, &next.axes[i], &next.povs[i], &next.buttons[i], &next.descriptors[i]);
    }
  }
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
  std::scoped_lock lock(m_optionsMutex);
  m_opModeIds.clear();
  for (int32_t i = 0; i < count; ++i) {
    m_opModeIds.push_back(options[i].id);
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

int32_t ProbotCoreDs::getSystemTimeValid(bool* systemTimeValid) {
  *systemTimeValid = true;  // the HAL clock is monotonic time since boot
  return 0;
}

int32_t ProbotCoreDs::writeDisplayAnsi(const struct WPI_String*) {
  return 0;  // no display
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
