// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetEncoderDrive: drive to a distance with an encoder (upstream
// wpilibcExamples/src/main/cpp/snippets/EncoderDrive), ported to
// prosdk.
//
// Wiring (GPIO numbers; channel = GPIO):
//   GPIO 6/7    encoder A/B
//   GPIO 4, 5   left leader / follower Spark (or any PWM ESC)
//   GPIO 12, 13 right leader / follower Spark

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drive/DifferentialDrive.hpp>
#include <prosdk/drivers/motor/Spark.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>

/**
 * Encoder drive to distance snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    // Configures the encoder's distance-per-pulse
    // The robot moves forward 1 foot per encoder rotation
    // There are 256 pulses per encoder rotation
    encoder.SetDistancePerPulse(1.0 / 256.0);
    // Invert the right side of the drivetrain. You might have to invert the
    // other side
    rightLeader.SetInverted(true);
    // Configure the followers to follow the leaders
    leftLeader.AddFollower(leftFollower);
    rightLeader.AddFollower(rightFollower);
  }
  void AutonomousPeriodic() override {
    // Drives forward at half velocity until the robot has moved 5 feet, then
    // stops:
    if (encoder.GetDistance() < 5) {
      drive.TankDrive(0.5, 0.5);
    } else {
      drive.TankDrive(0, 0);
    }
  }

 private:
  // Creates an encoder on GPIO 6 and 7.
  prosdk::Encoder encoder{6, 7};
  // Initialize motor controllers and drive
  prosdk::Spark leftLeader{4};
  prosdk::Spark leftFollower{5};
  prosdk::Spark rightLeader{12};
  prosdk::Spark rightFollower{13};
  prosdk::DifferentialDrive drive{
      [&](double output) { leftLeader.SetThrottle(output); },
      [&](double output) { rightLeader.SetThrottle(output); }};
};

PROSDK_ROBOT_MAIN(Robot)
