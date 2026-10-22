// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetDutyCycleInput: frequency and duty cycle of a PWM signal (upstream
// wpilibcExamples/src/main/cpp/snippets/DutyCycleInput),
// ported to prosdk. Values go to /Telemetry.
//
// Wiring (GPIO numbers; channel = GPIO): GPIO 17, a 3.3 V PWM signal.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/rotation/DutyCycle.hpp>
#include <prosdk/telemetry/Telemetry.hpp>

class Robot : public prosdk::TimedRobot {
  prosdk::DutyCycle dutyCycle{17};  // Duty cycle input

 public:
  Robot() {}

  void RobotPeriodic() override {
    // Duty Cycle Frequency in Hz
    auto frequency = dutyCycle.GetFrequency();

    // Output of duty cycle, ranging from 0 to 1
    // 1 is fully on, 0 is fully off
    auto output = dutyCycle.GetOutput();

    prosdk::telemetry::Log("Frequency", frequency);
    prosdk::telemetry::Log("Duty Cycle", output);
  }
};

PROSDK_ROBOT_MAIN(Robot)
