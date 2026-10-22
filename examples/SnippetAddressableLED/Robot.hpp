// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/led/AddressableLED.hpp>
#include <prosdk/hardware/led/LEDPattern.hpp>

class Robot : public prosdk::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;

 private:
  static constexpr int LENGTH = 60;

  // GPIO 21 (upstream: SmartIO port 1)
  prosdk::AddressableLED led{21};
  std::array<prosdk::AddressableLED::LEDData, LENGTH>
      ledBuffer;  // Reuse the buffer

  // Our LED strip has a density of 120 LEDs per meter
  prosdk::units::meter_t LED_SPACING{1 / 120.0};

  // Create an LED pattern that will display a rainbow across
  // all hues at maximum saturation and half brightness
  prosdk::LEDPattern rainbow = prosdk::LEDPattern::Rainbow(255, 128);

  // Create a new pattern that scrolls the rainbow pattern across the LED
  // strip, moving at a velocity of 1 meter per second.
  prosdk::LEDPattern scrollingRainbow =
      rainbow.ScrollAtAbsoluteVelocity(1_mps, LED_SPACING);
};
