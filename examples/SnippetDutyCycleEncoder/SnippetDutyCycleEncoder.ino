// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetDutyCycleEncoder: DutyCycleEncoder snippets (upstream
// wpilibcExamples/src/main/cpp/snippets/DutyCycleEncoder), ported to
// prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): GPIO 17 and GPIO 18, absolute (PWM output) encoders
// such as a REV Through Bore or CTRE Mag Encoder.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/rotation/DutyCycleEncoder.hpp>

/**
 * DutyCycleEncoder snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  Robot() {}

  void TeleopPeriodic() override {
    // Gets the rotation
    encoder.Get();

    // Gets if the encoder is connected
    encoder.IsConnected();
  }

 private:
  // Initializes a duty cycle encoder on GPIO 17
  prosdk::DutyCycleEncoder encoder{17};

  // Initializes a duty cycle encoder on GPIO 18 to return a value of 4 for
  // a full rotation, with the encoder reporting 0 half way through rotation (2
  // out of 4)
  prosdk::DutyCycleEncoder encoderFR{18, 4.0, 2.0};
};

PROSDK_ROBOT_MAIN(Robot)
