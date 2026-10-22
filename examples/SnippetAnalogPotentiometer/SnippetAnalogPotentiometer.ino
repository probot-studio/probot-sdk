// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetAnalogPotentiometer: AnalogPotentiometer snippets (upstream
// wpilibcExamples/src/main/cpp/snippets/AnalogPotentiometer),
// ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO, analog inputs are GPIO 1-10): potentiometer wipers on
// GPIO 1 and GPIO 2, ends to 3.3 V and GND.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/discrete/AnalogInput.hpp>
#include <prosdk/hardware/rotation/AnalogPotentiometer.hpp>

/**
 * AnalogPotentiometer snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/analog-potentiometers-software.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  Robot() {}

  void TeleopPeriodic() override {
    // Get the value of the potentiometer
    pot.Get();
  }

 private:
  // Initializes an AnalogPotentiometer on analog GPIO 1
  // The full range of motion (in meaningful external units) is 0-180 (this
  // could be degrees, for instance) The "starting point" of the motion, i.e.
  // where the mechanism is located when the potentiometer reads 0v, is 30.
  prosdk::AnalogPotentiometer pot{1, 180, 30};

  // Initializes an AnalogInput on GPIO 2
  prosdk::AnalogInput input{2};
  // Initializes an AnalogPotentiometer with the given AnalogInput
  // The full range of motion (in meaningful external units) is 0-180 (this
  // could be degrees, for instance) The "starting point" of the motion, i.e.
  // where the mechanism is located when the potentiometer reads 0v, is 30.
  prosdk::AnalogPotentiometer pot1{&input, 180, 30};
};

PROSDK_ROBOT_MAIN(Robot)
