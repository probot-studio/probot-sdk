// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetAnalogInput: AnalogInput snippets (upstream
// wpilibcExamples/src/main/cpp/snippets/AnalogInput), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO, analog inputs are GPIO 1-10): GPIO 1, 0-3.1 V.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/discrete/AnalogInput.hpp>

/**
 * AnalogInput snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/analog-input-software.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    // Gets the raw instantaneous measured value from the analog input, without
    // applying any calibration and ignoring oversampling and averaging
    // settings.
    analog.GetValue();

    // Gets the instantaneous measured voltage from the analog input.
    // Oversampling and averaging settings are ignored
    analog.GetVoltage();
  }

  void TeleopPeriodic() override {}

 private:
  // Initializes an AnalogInput on GPIO 1
  prosdk::AnalogInput analog{1};
};

PROSDK_ROBOT_MAIN(Robot)
