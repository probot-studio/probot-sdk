// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <stdexcept>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "prosdk/hal/HAL.h"
#include "prosdk/simulation/AddressableLEDSim.hpp"
#include "prosdk/simulation/AnalogInputSim.hpp"
#include "prosdk/simulation/CTREPCMSim.hpp"
#include "prosdk/simulation/DIOSim.hpp"
#include "prosdk/simulation/DigitalPWMSim.hpp"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/simulation/DutyCycleSim.hpp"
#include "prosdk/simulation/EncoderSim.hpp"
#include "prosdk/simulation/PWMSim.hpp"
#include "prosdk/simulation/PowerDistributionSim.hpp"
#include "prosdk/simulation/RoboRioSim.hpp"

using namespace prosdk::sim;

TEST_CASE("SimInitializationTest AllInitialize", "[wpilibc][simulation]") {
  HAL_Initialize();
  AnalogInputSim aisim{0};
  CHECK_THROWS_AS(DigitalPWMSim::CreateForChannel(0), std::out_of_range);
  DIOSim diosim{0};
  DriverStationSim dssim;
  (void)dssim;
  EncoderSim esim = EncoderSim::CreateForIndex(0);
  (void)esim;
  CTREPCMSim pcmsim{0};
  PowerDistributionSim pdpsim{0};
  PWMSim pwmsim{0};
  RoboRioSim rrsim;
  (void)rrsim;
  DutyCycleSim dcsim = DutyCycleSim::CreateForChannel(0);
  (void)dcsim;
  AddressableLEDSim adLED{0};
}
