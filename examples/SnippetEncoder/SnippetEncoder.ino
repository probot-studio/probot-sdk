// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetEncoder: Encoder snippets (upstream wpilibcExamples/src/main/cpp/snippets/Encoder), ported
// to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO):
//   GPIO 6/7    quadrature encoder A/B (4X decoding)
//   GPIO 15/16  second encoder A/B (2X decoding; upstream puts both on DIO 0/1, which cannot
//               be allocated twice)

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/util/deprecated.hpp>

/**
 * Encoder snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html
 */
WPI_IGNORE_DEPRECATED
class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    // Configures the encoder to return a distance of 4 for every 256 pulses
    // Also changes the units of getRate
    encoder.SetDistancePerPulse(4.0 / 256.0);
    // Reverses the direction of the encoder
    encoder.SetReverseDirection(true);
  }

  void TeleopPeriodic() override {
    // Gets the distance traveled
    encoder.GetDistance();

    // Gets the current rate of the encoder
    encoder.GetRate();

    // Gets whether the encoder is stopped
    encoder.GetStopped();

    // Gets the last direction in which the encoder moved
    encoder.GetDirection();

    // Gets the current period of the encoder

    // Resets the encoder to read a distance of zero
    encoder.Reset();
  }

 private:
  // Initializes an encoder on GPIO 6 and 7
  // Defaults to 4X decoding and non-inverted
  prosdk::Encoder encoder{6, 7};

  // Initializes an encoder on GPIO 15 and 16
  // 2X encoding and non-inverted
  prosdk::Encoder encoder2x{15, 16, false, prosdk::Encoder::EncodingType::X2};
};

PROSDK_ROBOT_MAIN(Robot)
