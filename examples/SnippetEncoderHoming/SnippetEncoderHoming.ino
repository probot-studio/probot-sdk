// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetEncoderHoming: home a mechanism on a limit switch (upstream
// wpilibcExamples/src/main/cpp/snippets/EncoderHoming), ported to
// prosdk.
//
// Wiring (GPIO numbers; channel = GPIO):
//   GPIO 6/7  encoder A/B         GPIO 4  Spark (or any PWM ESC)
//   GPIO 17   limit switch to GND (internal pull-up: open reads true)

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drivers/motor/Spark.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/discrete/DigitalInput.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>

/**
 * Encoder mechanism homing snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  void AutonomousPeriodic() override {
    // Runs the motor backwards at half velocity until the limit switch is
    // pressed then turn off the motor and reset the encoder
    if (!limit.Get()) {
      spark.SetThrottle(-0.5);
    } else {
      spark.SetThrottle(0);
      encoder.Reset();
    }
  }

 private:
  prosdk::Encoder encoder{6, 7};
  prosdk::Spark spark{4};
  // Limit switch on GPIO 17
  prosdk::DigitalInput limit{17};
};

PROSDK_ROBOT_MAIN(Robot)
