// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// probot-core's own gamepad mapping profiles (probot/io/joystick_mappings/mapping.hpp) through the
// bridge's copy (CopyCoreMapping) and the HAL conversion (prosdk/platform/hal/CoreJoystick.hpp):
// the centre Home/Guide/PS button is WPILib's Gamepad GUIDE button where core names it (R20:
// Options = W3C standard button 16 in logitech-f310, standard and axis9-dpad), and stays
// unavailable where it does not (a core before R20). Own executable: built with the probot-core
// clone's src on the include path ($PROBOT_CORE, default .upstream/probot-core,
// PROSDK_TEST_PROBOT_CORE_GAMEPAD); without a clone it has no cases.

#include <cstdint>
#include <cstring>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/hal/CoreJoystick.hpp"

#if PROSDK_TEST_PROBOT_CORE_GAMEPAD
#include <probot/io/joystick_mappings/mapping.hpp>

namespace pf = prosdk::platform;
namespace jm = probot::io::joystick_mapping;

namespace {

// a frame as the Driver Station page sends it for a W3C standard gamepad (17 buttons), with the
// buttons in `pressed` held
pf::CoreGamepad Frame(const jm::MappingProfile& profile, uint32_t pressed, uint8_t buttons = 17) {
  pf::CoreGamepad pad;
  std::memset(&pad, 0, sizeof(pad));
  pad.seq = 1;
  pad.axisCount = 4;
  pad.buttonCount = buttons;
  pad.buttons = pressed;
  pad.pov = -1;
  pad.standardMapping = 1;
  pf::CopyCoreMapping(profile, &pad.mapping);
  return pad;
}

bool Available(const pf::CoreJoystickData& d, int button) {
  return ((d.buttons.available >> button) & 1u) != 0;
}

bool Pressed(const pf::CoreJoystickData& d, int button) {
  return ((d.buttons.buttons >> button) & 1u) != 0;
}

}  // namespace

TEST_CASE("Core's profiles: the Home/Guide button is GUIDE where core names it",
          "[prosdk][hal][joystick]") {
  const jm::MappingProfile* standardProfiles[] = {
      &jm::mapping_logitech_f310(), &jm::mapping_standard(), &jm::mapping_axis9_dpad(),
      jm::findByName("xbox"),       jm::findByName("ps"),
  };
  // the clone decides which behavior is checked: CI and the pin have R20 (feat/gamepad-guide)
  const bool r20 = jm::mapping_standard().Options >= 0;
  INFO("probot-core clone " << (r20 ? "with" : "without") << " R20");
  for (const auto* profile : standardProfiles) {
    CAPTURE(profile->name);
    pf::CoreJoystickData d;
    pf::ConvertCoreGamepad(Frame(*profile, 1u << 16), &d);
    if (r20) {
      CHECK(profile->Options == 16);
      CHECK(Available(d, pf::kCoreJoystickGuide));
      CHECK(Pressed(d, pf::kCoreJoystickGuide));
    } else {
      CHECK(profile->Options == -1);
      CHECK_FALSE(Available(d, pf::kCoreJoystickGuide));
    }
    // Home is none of the other buttons
    CHECK(d.buttons.buttons == (r20 ? (uint64_t{1} << pf::kCoreJoystickGuide) : 0));
    // a pad that reports 16 buttons has no Home button
    pf::ConvertCoreGamepad(Frame(*profile, 0xffffu, 16), &d);
    CHECK_FALSE(Available(d, pf::kCoreJoystickGuide));
    CHECK_FALSE(Pressed(d, pf::kCoreJoystickGuide));
  }

  // tuna-default has had its own Options (10) all along
  pf::CoreJoystickData d;
  pf::ConvertCoreGamepad(Frame(jm::mapping_tuna_default(), 1u << 10), &d);
  CHECK(Available(d, pf::kCoreJoystickGuide));
  CHECK(Pressed(d, pf::kCoreJoystickGuide));
}
#endif
