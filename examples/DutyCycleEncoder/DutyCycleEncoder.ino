// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// DutyCycleEncoder: an absolute duty-cycle encoder with range, zero offset and wrap shift
// (upstream wpilibcExamples/src/main/cpp/examples/DutyCycleEncoder), ported to prosdk. Values go to
// /Telemetry.
//
// Wiring (GPIO numbers; channel = GPIO): GPIO 17, PWM output of the encoder (3.3 V).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/rotation/DutyCycleEncoder.hpp>
#include <prosdk/math/util/MathUtil.hpp>
#include <prosdk/telemetry/Telemetry.hpp>

constexpr double fullRange = 1.3;
constexpr double expectedZero = 0.0;

class Robot : public prosdk::TimedRobot {
  // 2nd parameter is the range of values. This sensor will output between
  // 0 and the passed in value.
  // 3rd parameter is the the physical value where you want "0" to be. How
  // to measure this is fairly easy. Set the value to 0, place the mechanism
  // where you want "0" to be, and observe the value on the dashboard, That
  // is the value to enter for the 3rd parameter.
  prosdk::DutyCycleEncoder dutyCycleEncoder{17, fullRange, expectedZero};

 public:
  Robot() {
    // If you know the frequency of your sensor, uncomment the following
    // method, and set the method to the frequency of your sensor.
    // This will result in more stable readings from the sensor.
    // Do note that occasionally the datasheet cannot be trusted
    // and you should measure this value. You can do so with either
    // an oscilloscope, or by observing the "Frequency" output
    // on the dashboard while running this sample. If you find
    // the value jumping between the 2 values, enter halfway between
    // those values. This number doesn't have to be perfect,
    // just having a fairly close value will make the output readings
    // much more stable.
    dutyCycleEncoder.SetAssumedFrequency(967.8_Hz);
  }

  void RobotPeriodic() override {
    // Connected can be checked, and uses the frequency of the encoder
    auto connected = dutyCycleEncoder.IsConnected();

    // Duty Cycle Frequency in Hz
    auto frequency = dutyCycleEncoder.GetFrequency();

    // Output of encoder
    auto output = dutyCycleEncoder.Get();

    // By default, the output will wrap around to the full range value
    // when the sensor goes below 0. However, for moving mechanisms this
    // is not usually ideal, as if 0 is set to a hard stop, its still
    // possible for the sensor to move slightly past. If this happens
    // The sensor will assume its now at the furthest away position,
    // which control algorithms might not handle correctly. Therefore
    // it can be a good idea to slightly shift the output so the sensor
    // can go a bit negative before wrapping. Usually 10% or so is fine.
    // This does not change where "0" is, so no calibration numbers need
    // to be changed.
    double percentOfRange = fullRange * 0.1;
    double shiftedOutput = prosdk::math::InputModulus(output, 0 - percentOfRange,
                                                   fullRange - percentOfRange);

    prosdk::telemetry::Log("Connected", connected);
    prosdk::telemetry::Log("Frequency", frequency);
    prosdk::telemetry::Log("Output", output);
    prosdk::telemetry::Log("ShiftedOutput", shiftedOutput);
  }
};

PROSDK_ROBOT_MAIN(Robot)
