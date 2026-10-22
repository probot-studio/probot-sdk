// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetAnalogEncoder: AnalogEncoder snippets (upstream
// wpilibcExamples/src/main/cpp/snippets/AnalogEncoder), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO, analog inputs are GPIO 1-10): GPIO 1 and GPIO 2,
// analog absolute encoders (0-3.1 V on the ESP32-S3 ADC).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/rotation/AnalogEncoder.hpp>

/**
 * AnalogEncoder snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  Robot() {}

  void TeleopPeriodic() override {
    // Gets the rotation
    encoder.Get();
  }

 private:
  // Initializes an analog encoder on analog input GPIO 1
  prosdk::AnalogEncoder encoder{1};

  // Initializes an analog encoder on analog input GPIO 2 to return a value of 4 for
  // a full rotation, with the encoder reporting 0 half way through rotation (2
  // out of 4)
  prosdk::AnalogEncoder encoderFR{2, 4.0, 2.0};
};

PROSDK_ROBOT_MAIN(Robot)
