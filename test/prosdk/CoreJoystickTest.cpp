// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// probot-core R8 gamepads -> WPILib HAL joysticks (prosdk/platform/hal/CoreJoystick.hpp), the
// conversion the ESP32 Driver Station backend runs for ports 0-5. A fake CoreOps table (not
// installed: CoreOpsTest owns the one install per process) stands in for the sketch-side bridge.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/hal/DriverStationTypes.h"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/hal/CoreJoystick.hpp"

namespace pf = prosdk::platform;
using Catch::Approx;

namespace {

// core's "standard" mapping profile (W3C standard gamepad order) as the bridge copies it
pf::CoreGamepadMapping StandardMapping() {
  pf::CoreGamepadMapping m{};
  m.name = "standard";
  m.leftX = 0;
  m.leftY = 1;
  m.rightX = 2;
  m.rightY = 3;
  m.invertLeftY = 1;
  m.invertRightY = 1;
  m.a = 0;
  m.b = 1;
  m.x = 2;
  m.y = 3;
  m.leftBumper = 4;
  m.rightBumper = 5;
  m.leftTrigger = 6;
  m.rightTrigger = 7;
  m.back = 8;
  m.start = 9;
  m.options = -1;
  m.leftStick = 10;
  m.rightStick = 11;
  m.dpadFromAxis = 0;
  m.dpadAxis = -1;
  m.dpadUp = 12;
  m.dpadDown = 13;
  m.dpadLeft = 14;
  m.dpadRight = 15;
  return m;
}

pf::CoreGamepad Pad(const char* name, bool standard) {
  pf::CoreGamepad pad;
  std::memset(&pad, 0, sizeof(pad));
  pad.seq = 1;
  pad.axisCount = 4;
  pad.buttonCount = 17;
  pad.mapping = StandardMapping();
  pad.pov = -1;
  pad.standardMapping = standard ? 1 : 0;
  std::snprintf(pad.name, sizeof(pad.name), "%s", name);
  return pad;
}

// fake bridge: ports 0, 3 and 5 received frames; port 5 was neutralized by core (stale)
pf::CoreGamepad gPorts[pf::kCoreMaxGamepads];
bool gHasFrame[pf::kCoreMaxGamepads];

bool FakeRawGamepad(int port, pf::CoreGamepad* out) {
  if (port < 0 || port >= pf::kCoreMaxGamepads || !gHasFrame[port]) {
    return false;
  }
  *out = gPorts[port];
  return true;
}

pf::CoreOps FakeOps() {
  pf::CoreOps ops{};
  ops.abiVersion = pf::kCoreOpsAbiVersion;
  ops.size = sizeof(pf::CoreOps);
  ops.rawGamepad = FakeRawGamepad;
  return ops;
}

bool Button(const pf::CoreJoystickData& d, int index) {
  return ((d.buttons.buttons >> index) & 1u) != 0;
}

}  // namespace

TEST_CASE("Core gamepads map to HAL joysticks 0-5", "[prosdk][hal][joystick]") {
  for (auto& has : gHasFrame) {
    has = false;
  }
  gPorts[0] = Pad("Xbox 360 Controller (XInput STANDARD GAMEPAD)", true);
  gPorts[0].axes[0] = 0.5f;
  gPorts[0].axes[1] = -0.25f;  // W3C: up is negative; core inverts, WPILib keeps it
  gPorts[0].buttons = (1u << 0) | (1u << 5);  // A, right bumper
  gHasFrame[0] = true;
  gPorts[3] = Pad("8BitDo Pro 2", false);
  gPorts[3].buttons = 1u << 3;  // Y
  gHasFrame[3] = true;
  gPorts[5] = Pad("", true);
  gPorts[5].axisCount = 0;
  gPorts[5].buttonCount = 0;
  gHasFrame[5] = true;
  const pf::CoreOps ops = FakeOps();

  SECTION("ports") {
    pf::CoreJoystickData d;
    REQUIRE(pf::ReadCoreJoystick(ops, 0, &d));
    CHECK(d.axes.available == 0x3f);  // sticks and both triggers
    CHECK(d.axes.axes[pf::kCoreJoystickLeftX] == Approx(0.5f));
    CHECK(d.axes.axes[pf::kCoreJoystickLeftY] == Approx(-0.25f));
    CHECK(Button(d, pf::kCoreJoystickFaceDown));
    CHECK(Button(d, pf::kCoreJoystickRightBumper));
    CHECK_FALSE(Button(d, pf::kCoreJoystickFaceUp));
    CHECK((d.buttons.available & 0x7fffu) == 0x7fdfu);  // 0-14 except GUIDE (no mapping)

    REQUIRE(pf::ReadCoreJoystick(ops, 3, &d));
    CHECK(Button(d, pf::kCoreJoystickFaceUp));
    CHECK_FALSE(Button(d, pf::kCoreJoystickFaceDown));

    // no frame on ports 1, 2, 4; port 5 neutralized; out of range
    for (int port : {1, 2, 4}) {
      CHECK_FALSE(pf::ReadCoreJoystick(ops, port, &d));
      CHECK(d.axes.available == 0);
      CHECK(d.buttons.available == 0);
      CHECK(d.povs.available == 0);
    }
    CHECK(pf::ReadCoreJoystick(ops, 5, &d));
    CHECK(d.axes.available == 0);
    CHECK(d.buttons.available == 0);
    CHECK(d.povs.available == 0);
    CHECK(d.descriptor.name[0] == '\0');
    CHECK_FALSE(pf::ReadCoreJoystick(ops, 6, &d));
    CHECK_FALSE(pf::ReadCoreJoystick(ops, -1, &d));
  }

  SECTION("analog triggers") {
    pf::CoreGamepad& pad = gPorts[0];
    pad.leftTrigger = 0.0f;  // core: raw 15/255 = 0.059 is inside the 0.08 deadzone
    pad.leftTriggerRaw = 15;
    pad.rightTrigger = 128.0f / 255.0f;
    pad.rightTriggerRaw = 128;
    pf::CoreJoystickData d;
    REQUIRE(pf::ReadCoreJoystick(ops, 0, &d));
    CHECK(d.axes.axes[pf::kCoreJoystickLeftTrigger] == 0.0f);
    CHECK(d.axes.raw[pf::kCoreJoystickLeftTrigger] == (15 * 32767 + 127) / 255);
    CHECK(d.axes.axes[pf::kCoreJoystickRightTrigger] == Approx(128.0 / 255.0));
    CHECK(d.axes.raw[pf::kCoreJoystickRightTrigger] == (128 * 32767 + 127) / 255);

    pad.rightTrigger = 1.0f;
    pad.rightTriggerRaw = 255;
    REQUIRE(pf::ReadCoreJoystick(ops, 0, &d));
    CHECK(d.axes.axes[pf::kCoreJoystickRightTrigger] == 1.0f);
    CHECK(d.axes.raw[pf::kCoreJoystickRightTrigger] == 32767);

    // a mapping whose trigger buttons the snapshot does not carry has no trigger axes
    pad.buttonCount = 6;
    REQUIRE(pf::ReadCoreJoystick(ops, 0, &d));
    CHECK(d.axes.available == 0x0f);
  }

  SECTION("8-way POV and D-pad buttons") {
    struct Case {
      int povDegrees;
      HAL_JoystickPOV pov;
      bool up, right, down, left;
    };
    const Case cases[] = {
        {-1, HAL_JOYSTICK_POV_CENTERED, false, false, false, false},
        {0, HAL_JOYSTICK_POV_UP, true, false, false, false},
        {45, HAL_JOYSTICK_POV_RIGHT_UP, true, true, false, false},
        {90, HAL_JOYSTICK_POV_RIGHT, false, true, false, false},
        {135, HAL_JOYSTICK_POV_RIGHT_DOWN, false, true, true, false},
        {180, HAL_JOYSTICK_POV_DOWN, false, false, true, false},
        {225, HAL_JOYSTICK_POV_LEFT_DOWN, false, false, true, true},
        {270, HAL_JOYSTICK_POV_LEFT, false, false, false, true},
        {315, HAL_JOYSTICK_POV_LEFT_UP, true, false, false, true},
        {30, HAL_JOYSTICK_POV_CENTERED, false, false, false, false},
    };
    for (const Case& c : cases) {
      CAPTURE(c.povDegrees);
      gPorts[3].pov = static_cast<int16_t>(c.povDegrees);
      pf::CoreJoystickData d;
      REQUIRE(pf::ReadCoreJoystick(ops, 3, &d));
      CHECK(d.povs.available == 1);
      CHECK(d.povs.povs[0] == c.pov);
      CHECK(Button(d, pf::kCoreJoystickDpadUp) == c.up);
      CHECK(Button(d, pf::kCoreJoystickDpadRight) == c.right);
      CHECK(Button(d, pf::kCoreJoystickDpadDown) == c.down);
      CHECK(Button(d, pf::kCoreJoystickDpadLeft) == c.left);
      CHECK((d.buttons.available >> pf::kCoreJoystickDpadUp & 0xfu) == 0xfu);
    }

    // hat-axis D-pad profile: available when the snapshot carries the hat axis
    gPorts[3].mapping.dpadFromAxis = 1;
    gPorts[3].mapping.dpadAxis = 9;
    gPorts[3].pov = 90;
    pf::CoreJoystickData d;
    REQUIRE(pf::ReadCoreJoystick(ops, 3, &d));
    CHECK(d.povs.available == 0);  // axisCount 4: no axis 9
    gPorts[3].axisCount = 10;
    REQUIRE(pf::ReadCoreJoystick(ops, 3, &d));
    CHECK(d.povs.available == 1);
    CHECK(d.povs.povs[0] == HAL_JOYSTICK_POV_RIGHT);
  }

  SECTION("lost joysticks") {
    pf::CoreJoystickData d;
    REQUIRE(pf::ReadCoreJoystick(ops, 0, &d));
    CHECK(pf::CoreJoystickConnected(d));
    REQUIRE(pf::ReadCoreJoystick(ops, 5, &d));
    CHECK_FALSE(pf::CoreJoystickConnected(d));  // neutralized: a frame, but no data
    // ports 0 and 3 connected in the last snapshot; nothing lost yet
    CHECK(pf::LostCoreJoysticks(ops, 0x09, &d) == 0);
    // core's input timeout neutralizes port 0 (no new packet); port 3 stays
    gPorts[0].axisCount = 0;
    gPorts[0].buttonCount = 0;
    CHECK(pf::LostCoreJoysticks(ops, 0x09, &d) == 0x01);
    // the Driver Station released the robot: no frame on port 3 any more
    gHasFrame[3] = false;
    CHECK(pf::LostCoreJoysticks(ops, 0x09, &d) == 0x09);
    // ports outside the connected mask are not checked
    CHECK(pf::LostCoreJoysticks(ops, 0x00, &d) == 0);
  }

  SECTION("name and gamepad type") {
    pf::CoreJoystickData d;
    REQUIRE(pf::ReadCoreJoystick(ops, 0, &d));
    CHECK(std::string{d.descriptor.name} == "Xbox 360 Controller (XInput STANDARD GAMEPAD)");
    CHECK(d.descriptor.isGamepad == 1);
    CHECK(d.descriptor.gamepadType == 1);  // GenericHID::HIDType::STANDARD
    CHECK(d.descriptor.supportedOutputs == 0);

    REQUIRE(pf::ReadCoreJoystick(ops, 3, &d));
    CHECK(std::string{d.descriptor.name} == "8BitDo Pro 2");
    CHECK(d.descriptor.isGamepad == 0);    // not the standard mapping: generic HID
    CHECK(d.descriptor.gamepadType == 0);  // HIDType::UNKNOWN

    // the longest name core sends (63 bytes of UTF-8) is kept whole; nothing longer is kept
    std::string longest;
    while (longest.size() + 2 <= pf::kCoreGamepadMaxName) {
      longest += "\xc3\xa7";  // U+00E7, two bytes
    }
    longest += "x";
    REQUIRE(longest.size() == pf::kCoreGamepadMaxName);
    std::memcpy(gPorts[0].name, longest.c_str(), longest.size() + 1);
    REQUIRE(pf::ReadCoreJoystick(ops, 0, &d));
    CHECK(std::string{d.descriptor.name} == longest);
    std::memset(gPorts[0].name, 'y', sizeof(gPorts[0].name));  // not terminated
    REQUIRE(pf::ReadCoreJoystick(ops, 0, &d));
    CHECK(std::string{d.descriptor.name}.size() == pf::kCoreGamepadMaxName);
  }
}

TEST_CASE("Core's Home/Guide button is the Gamepad GUIDE button", "[prosdk][hal][joystick]") {
  // probot-core R20: Options is W3C standard button 16 (the centre Home/Guide/PS button) in the
  // logitech-f310, standard and axis9-dpad profiles; -1 there before R20
  pf::CoreGamepad pad = Pad("Xbox Wireless Controller (STANDARD GAMEPAD)", true);
  pad.mapping.options = 16;
  pad.buttons = 1u << 16;
  pf::CoreJoystickData d;
  pf::ConvertCoreGamepad(pad, &d);
  CHECK((d.buttons.available & 0x7fffu) == 0x7fffu);  // all 15 Gamepad buttons, GUIDE too
  CHECK(Button(d, pf::kCoreJoystickGuide));
  CHECK_FALSE(Button(d, pf::kCoreJoystickBack));
  CHECK_FALSE(Button(d, pf::kCoreJoystickStart));
  pad.buttons = 0;
  pf::ConvertCoreGamepad(pad, &d);
  CHECK((d.buttons.available >> pf::kCoreJoystickGuide & 1u) == 1u);
  CHECK_FALSE(Button(d, pf::kCoreJoystickGuide));

  // a pad that reports 16 buttons has no Home button: GUIDE not available
  pad.buttonCount = 16;
  pad.buttons = 0xffffu;
  pf::ConvertCoreGamepad(pad, &d);
  CHECK((d.buttons.available >> pf::kCoreJoystickGuide & 1u) == 0u);
  CHECK_FALSE(Button(d, pf::kCoreJoystickGuide));

  // a core before R20 (Options -1 in the standard profiles): GUIDE not available, even when the
  // frame carries button 16
  pad.buttonCount = 17;
  pad.buttons = 1u << 16;
  pad.mapping.options = -1;
  pf::ConvertCoreGamepad(pad, &d);
  CHECK((d.buttons.available & 0x7fffu) == 0x7fdfu);
  CHECK_FALSE(Button(d, pf::kCoreJoystickGuide));

  // core's tuna-default profile has had Options = 10 all along
  pad.mapping.options = 10;
  pad.buttons = 1u << 10;
  pf::ConvertCoreGamepad(pad, &d);
  CHECK(Button(d, pf::kCoreJoystickGuide));
}

TEST_CASE("CopyCoreMapping copies every field of core's profile", "[prosdk][hal][joystick]") {
  // core's MappingProfile layout (probot/io/joystick_mappings/mapping.hpp)
  struct Profile {
    const char* name;
    int leftX, leftY, rightX, rightY;
    bool invertLeftY, invertRightY;
    int A, B, X, Y, LB, RB, LT, RT, Back, Start, Options, LStick, RStick;
    bool dpadFromAxis;
    int dpadAxis;
    int DPadUp, DPadDown, DPadLeft, DPadRight;
  };
  const Profile standard{"standard", 0, 1, 2, 3, true, true, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 16,
                         10, 11, false, -1, 12, 13, 14, 15};
  pf::CoreGamepadMapping m;
  std::memset(&m, 0x55, sizeof(m));
  pf::CopyCoreMapping(standard, &m);
  pf::CoreGamepadMapping want = StandardMapping();
  want.options = 16;
  CHECK(std::string{m.name} == want.name);
  auto fields = [](const pf::CoreGamepadMapping& g) {
    return std::vector<int>{g.leftX,        g.leftY,       g.rightX,      g.rightY,
                            g.invertLeftY,  g.invertRightY, g.a,          g.b,
                            g.x,            g.y,           g.leftBumper,  g.rightBumper,
                            g.leftTrigger,  g.rightTrigger, g.back,       g.start,
                            g.options,      g.leftStick,   g.rightStick,  g.dpadFromAxis,
                            g.dpadAxis,     g.dpadUp,      g.dpadDown,    g.dpadLeft,
                            g.dpadRight};
  };
  CHECK(fields(m) == fields(want));

  // every field a value of its own (the standard profile repeats 0-3 in sticks and face buttons,
  // so a field copied from the wrong source would not show there)
  const Profile distinct{"distinct", 20, 21, 22, 23, true, false, 30, 31, 32, 33, 34, 35, 36, 37,
                         38, 39, 40, 41, 42, true, 50, 51, 52, 53, 54};
  std::memset(&m, 0x55, sizeof(m));
  pf::CopyCoreMapping(distinct, &m);
  CHECK(std::string{m.name} == "distinct");
  CHECK(fields(m) == std::vector<int>{20, 21, 22, 23, 1, 0, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
                                      40, 41, 42, 1, 50, 51, 52, 53, 54});
  const Profile flipped{"flipped", 20, 21, 22, 23, false, true, 30, 31, 32, 33, 34, 35, 36, 37,
                        38, 39, 40, 41, 42, false, 50, 51, 52, 53, 54};
  pf::CopyCoreMapping(flipped, &m);
  CHECK(m.invertLeftY == 0);
  CHECK(m.invertRightY == 1);
  CHECK(m.dpadFromAxis == 0);

  // a null name reads as "", an index outside int8_t as none
  Profile odd = standard;
  odd.name = nullptr;
  odd.Options = 200;
  odd.LT = -5;
  pf::CopyCoreMapping(odd, &m);
  CHECK(std::string{m.name}.empty());
  CHECK(m.options == -1);
  CHECK(m.leftTrigger == -1);
}
