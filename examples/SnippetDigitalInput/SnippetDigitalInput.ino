// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetDigitalInput: DigitalInput snippets (upstream
// wpilibcExamples/src/main/cpp/snippets/DigitalInput), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): GPIO 17, a switch to GND (internal pull-up).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/discrete/DigitalInput.hpp>

/**
 * Digital Input snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/digital-input-software.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  void TeleopPeriodic() override {
    // Gets the value of the digital input.  Returns true if the circuit is
    // open.
    input.Get();
  }

 private:
  // Initializes a DigitalInput on GPIO 17
  prosdk::DigitalInput input{17};
};

PROSDK_ROBOT_MAIN(Robot)
