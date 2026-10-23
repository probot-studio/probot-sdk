// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// probot-core's own gamepad mapping profiles (probot/io/joystick_mappings/mapping.hpp) through the
// bridge's copy (CopyCoreMapping) and the HAL conversion (prosdk/platform/hal/CoreJoystick.hpp):
// the centre Home/Guide/PS button is WPILib's Gamepad GUIDE button where core names it (R20:
// Options = W3C standard button 16 in logitech-f310, standard and axis9-dpad), and stays
// unavailable where it does not (a core before R20). Own executable: built with the probot-core
// clone's src on the include path ($PROBOT_CORE, default .upstream/probot-core,
// PROSDK_TEST_PROBOT_CORE_GAMEPAD).
//   - The expected core is set by the build, not read from the clone: R20 (CI's PROBOT_CORE_REF and
//     the pinned clone), or a core before R20 when configured with PROBOT_CORE_BEFORE_R20=1 in the
//     environment (PROSDK_TEST_CORE_BEFORE_R20). A clone that does not match fails.
//   - Without a clone the one case only warns that core's profiles are untested.

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
  // the build says which core this is: R20 or later (CI and the pin are on a core with R20) unless
  // configured for an older one
#if PROSDK_TEST_CORE_BEFORE_R20
  constexpr bool r20 = false;
#else
  constexpr bool r20 = true;
#endif
  INFO("expecting a probot-core " << (r20 ? "with" : "before") << " R20 (PROBOT_CORE_BEFORE_R20)");
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
#else
TEST_CASE("Core's profiles: no probot-core clone", "[prosdk][hal][joystick]") {
  WARN("no probot-core clone at $PROBOT_CORE: core's gamepad profiles are untested");
  SUCCEED();
}
#endif
