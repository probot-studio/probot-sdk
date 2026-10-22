// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetAnalogAccelerometer: AnalogAccelerometer snippets (upstream
// wpilibcExamples/src/main/cpp/snippets/AnalogAccelerometer),
// ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO, analog inputs are GPIO 1-10): accelerometer output on
// GPIO 1 (the ESP32-S3 ADC reads 0-3.1 V; the 3 V zero of the upstream snippet is near its top).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/accelerometer/AnalogAccelerometer.hpp>
#include <prosdk/hardware/discrete/AnalogInput.hpp>

/**
 * AnalogAccelerometer snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/accelerometers-software.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    // Sets the sensitivity of the accelerometer to 1 volt per G
    accelerometer.SetSensitivity(1);
    // Sets the zero voltage of the accelerometer to 3 volts
    accelerometer.SetZero(3);
  }

  void TeleopPeriodic() override {
    // Gets the current acceleration
    accelerometer.GetAcceleration();
  }

 private:
  // Creates an analog accelerometer on analog input GPIO 1
  prosdk::AnalogAccelerometer accelerometer{1};
};

PROSDK_ROBOT_MAIN(Robot)
