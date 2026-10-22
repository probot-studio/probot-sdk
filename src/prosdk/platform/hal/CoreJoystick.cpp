// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/hal/CoreJoystick.hpp"

#include <cstring>

namespace prosdk::platform {

namespace {

bool Has(const CoreGamepad& pad, int index) {
  return index >= 0 && index < pad.buttonCount && index < 32;
}

bool RawButton(const CoreGamepad& pad, int index) {
  return Has(pad, index) && ((pad.buttons >> index) & 1u);
}

// Raw axis (W3C, -1..1); false when the mapping has no such axis.
bool RawAxis(const CoreGamepad& pad, int index, float* value) {
  if (index < 0 || index >= pad.axisCount || index >= kCoreGamepadMaxAxes) {
    return false;
  }
  *value = pad.axes[index];
  return true;
}

float Clamp(float v, float lo, float hi) {
  // NaN -> lo
  return v > hi ? hi : (v >= lo ? v : lo);
}

void SetAxis(HAL_JoystickAxes* axes, int index, float v, int16_t raw) {
  axes->axes[index] = v;
  axes->raw[index] = raw;
  axes->available |= static_cast<uint16_t>(1u << index);
}

void SetStick(HAL_JoystickAxes* axes, int index, float v) {
  v = Clamp(v, -1.0f, 1.0f);
  SetAxis(axes, index, v, static_cast<int16_t>(v * 32767.0f));
}

void SetTrigger(HAL_JoystickAxes* axes, int index, float v, uint8_t raw) {
  SetAxis(axes, index, Clamp(v, 0.0f, 1.0f),
          static_cast<int16_t>((static_cast<int32_t>(raw) * 32767 + 127) / 255));
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

}  // namespace

HAL_JoystickPOV CorePovToHal(int povDegrees) noexcept {
  switch (povDegrees) {
    case 0:
      return HAL_JOYSTICK_POV_UP;
    case 45:
      return HAL_JOYSTICK_POV_RIGHT_UP;
    case 90:
      return HAL_JOYSTICK_POV_RIGHT;
    case 135:
      return HAL_JOYSTICK_POV_RIGHT_DOWN;
    case 180:
      return HAL_JOYSTICK_POV_DOWN;
    case 225:
      return HAL_JOYSTICK_POV_LEFT_DOWN;
    case 270:
      return HAL_JOYSTICK_POV_LEFT;
    case 315:
      return HAL_JOYSTICK_POV_LEFT_UP;
    default:
      return HAL_JOYSTICK_POV_CENTERED;
  }
}

void ConvertCoreGamepad(const CoreGamepad& pad, CoreJoystickData* out) noexcept {
  std::memset(out, 0, sizeof(*out));
  if (pad.axisCount == 0 && pad.buttonCount == 0) {
    return;  // neutralized by core: no data, disconnected
  }
  HAL_JoystickAxes* axes = &out->axes;
  HAL_JoystickButtons* buttons = &out->buttons;
  const auto& m = pad.mapping;

  float v = 0;
  if (RawAxis(pad, m.leftX, &v)) {
    SetStick(axes, kCoreJoystickLeftX, v);
  }
  if (RawAxis(pad, m.leftY, &v)) {
    // raw is W3C (up negative) when core inverts it; WPILib/SDL is also up negative
    SetStick(axes, kCoreJoystickLeftY, m.invertLeftY ? v : -v);
  }
  if (RawAxis(pad, m.rightX, &v)) {
    SetStick(axes, kCoreJoystickRightX, v);
  }
  if (RawAxis(pad, m.rightY, &v)) {
    SetStick(axes, kCoreJoystickRightY, m.invertRightY ? v : -v);
  }
  if (Has(pad, m.leftTrigger)) {
    SetTrigger(axes, kCoreJoystickLeftTrigger, pad.leftTrigger, pad.leftTriggerRaw);
  }
  if (Has(pad, m.rightTrigger)) {
    SetTrigger(axes, kCoreJoystickRightTrigger, pad.rightTrigger, pad.rightTriggerRaw);
  }

  SetButton(buttons, kCoreJoystickFaceDown, Has(pad, m.a), RawButton(pad, m.a));
  SetButton(buttons, kCoreJoystickFaceRight, Has(pad, m.b), RawButton(pad, m.b));
  SetButton(buttons, kCoreJoystickFaceLeft, Has(pad, m.x), RawButton(pad, m.x));
  SetButton(buttons, kCoreJoystickFaceUp, Has(pad, m.y), RawButton(pad, m.y));
  SetButton(buttons, kCoreJoystickBack, Has(pad, m.back), RawButton(pad, m.back));
  SetButton(buttons, kCoreJoystickGuide, Has(pad, m.options), RawButton(pad, m.options));
  SetButton(buttons, kCoreJoystickStart, Has(pad, m.start), RawButton(pad, m.start));
  SetButton(buttons, kCoreJoystickLeftStick, Has(pad, m.leftStick), RawButton(pad, m.leftStick));
  SetButton(buttons, kCoreJoystickRightStick, Has(pad, m.rightStick),
            RawButton(pad, m.rightStick));
  SetButton(buttons, kCoreJoystickLeftBumper, Has(pad, m.leftBumper),
            RawButton(pad, m.leftBumper));
  SetButton(buttons, kCoreJoystickRightBumper, Has(pad, m.rightBumper),
            RawButton(pad, m.rightBumper));

  // D-pad: a hat axis the snapshot carries, or any of the four D-pad buttons
  float hat = 0;
  const bool dpad = m.dpadFromAxis ? RawAxis(pad, m.dpadAxis, &hat)
                                   : (Has(pad, m.dpadUp) || Has(pad, m.dpadDown) ||
                                      Has(pad, m.dpadLeft) || Has(pad, m.dpadRight));
  if (dpad) {
    const HAL_JoystickPOV pov = CorePovToHal(pad.pov);
    SetButton(buttons, kCoreJoystickDpadUp, true, (pov & HAL_JOYSTICK_POV_UP) != 0);
    SetButton(buttons, kCoreJoystickDpadDown, true, (pov & HAL_JOYSTICK_POV_DOWN) != 0);
    SetButton(buttons, kCoreJoystickDpadLeft, true, (pov & HAL_JOYSTICK_POV_LEFT) != 0);
    SetButton(buttons, kCoreJoystickDpadRight, true, (pov & HAL_JOYSTICK_POV_RIGHT) != 0);
    out->povs.povs[0] = pov;
    out->povs.available = 1;
  }

  HAL_JoystickDescriptor& desc = out->descriptor;
  // GenericHID::HIDType::STANDARD (1) for the W3C standard mapping, otherwise a generic HID
  desc.isGamepad = pad.standardMapping ? 1 : 0;
  desc.gamepadType = pad.standardMapping ? 1 : 0;
  desc.supportedOutputs = 0;  // no rumble, no LEDs
  size_t n = strnlen(pad.name, sizeof(pad.name));
  if (n > kCoreGamepadMaxName) {
    n = kCoreGamepadMaxName;
  }
  std::memcpy(desc.name, pad.name, n);
  desc.name[n] = '\0';
}

bool CoreJoystickConnected(const CoreJoystickData& data) noexcept {
  return data.axes.available != 0 || data.buttons.available != 0 || data.povs.available != 0;
}

uint8_t LostCoreJoysticks(const CoreOps& ops, uint8_t connected,
                          CoreJoystickData* scratch) noexcept {
  uint8_t lost = 0;
  for (int port = 0; port < kCoreMaxGamepads; ++port) {
    const uint8_t bit = static_cast<uint8_t>(1u << port);
    if ((connected & bit) != 0 &&
        !(ReadCoreJoystick(ops, port, scratch) && CoreJoystickConnected(*scratch))) {
      lost |= bit;
    }
  }
  return lost;
}

bool ReadCoreJoystick(const CoreOps& ops, int port, CoreJoystickData* out) noexcept {
  std::memset(out, 0, sizeof(*out));
  if (port < 0 || port >= kCoreMaxGamepads || ops.rawGamepad == nullptr) {
    return false;
  }
  CoreGamepad pad;
  std::memset(&pad, 0, sizeof(pad));
  pad.pov = -1;
  if (!ops.rawGamepad(port, &pad)) {
    return false;
  }
  ConvertCoreGamepad(pad, out);
  return true;
}

}  // namespace prosdk::platform
