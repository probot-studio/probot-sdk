// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/analog/AnalogCurrentSensor.hpp"

#include <limits>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/simulation/AnalogInputSim.hpp"

using namespace prosdk;

TEST_CASE("AnalogCurrentSensor INA250A1 preset", "[analogcurrent]") {
  sim::AnalogInputSim adc{1};
  adc.SetVoltage(0.0);

  // REF grounded (unidirectional): 200 mV/A from 0 V
  AnalogCurrentSensor ina = AnalogCurrentSensor::INA250A1(1, 0.0);
  CHECK(ina.GetChannel() == 1);
  CHECK(ina.GetCurrent().value() == Catch::Approx(0.0).margin(1e-9));
  adc.SetVoltage(1.0);
  CHECK(ina.GetCurrent().value() == Catch::Approx(5.0));
  CHECK(ina.GetVoltage().value() == Catch::Approx(1.0));
  adc.SetVoltage(0.1);
  CHECK(ina.GetCurrent().value() == Catch::Approx(0.5));
}

TEST_CASE("AnalogCurrentSensor INA250A1 bidirectional (REF at mid-rail)", "[analogcurrent]") {
  sim::AnalogInputSim adc{2};
  adc.SetVoltage(0.0);

  // REF at 1.65 V: below REF the current is negative
  AnalogCurrentSensor bidir = AnalogCurrentSensor::INA250A1(2, 1.65);
  adc.SetVoltage(1.25);
  CHECK(bidir.GetCurrent().value() == Catch::Approx(-2.0));
  adc.SetVoltage(2.05);
  CHECK(bidir.GetCurrent().value() == Catch::Approx(2.0));
}

TEST_CASE("AnalogCurrentSensor INA250A1 with a divider", "[analogcurrent]") {
  sim::AnalogInputSim adc{3};
  adc.SetVoltage(0.0);

  // VS = 5 V behind a 2:1 divider: 1 V at the pin is 2 V at OUT
  AnalogCurrentSensor ina = AnalogCurrentSensor::INA250A1(3, 0.0, 2.0);
  adc.SetVoltage(1.0);
  CHECK(ina.GetVoltage().value() == Catch::Approx(2.0));
  CHECK(ina.GetCurrent().value() == Catch::Approx(10.0));
}

TEST_CASE("AnalogCurrentSensor ACS758LCB-050B presets", "[analogcurrent]") {
  sim::AnalogInputSim adc{4};
  adc.SetVoltage(0.0);

  SECTION("VCC 5 V behind a 1.5 divider: 40 mV/A around 2.5 V") {
    AnalogCurrentSensor acs = AnalogCurrentSensor::ACS758LCB050B(4, 1.5);
    CHECK(acs.GetZeroVoltage().value() == Catch::Approx(2.5));
    adc.SetVoltage(2.5 / 1.5);
    CHECK(acs.GetCurrent().value() == Catch::Approx(0.0).margin(1e-9));
    adc.SetVoltage(2.9 / 1.5);
    CHECK(acs.GetCurrent().value() == Catch::Approx(10.0));
    adc.SetVoltage(2.1 / 1.5);
    CHECK(acs.GetCurrent().value() == Catch::Approx(-10.0));
  }

  SECTION("VCC 3.3 V, no divider: ratiometric 26.4 mV/A around 1.65 V") {
    AnalogCurrentSensor acs = AnalogCurrentSensor::ACS758LCB050B(4, 1.0, 3.3);
    CHECK(acs.GetZeroVoltage().value() == Catch::Approx(1.65));
    adc.SetVoltage(1.65 + 0.264);
    CHECK(acs.GetCurrent().value() == Catch::Approx(10.0));
  }
}

TEST_CASE("AnalogCurrentSensor CalibrateZero", "[analogcurrent]") {
  sim::AnalogInputSim adc{5};
  adc.SetVoltage(0.0);

  AnalogCurrentSensor acs = AnalogCurrentSensor::ACS758LCB050B(5, 1.5);
  adc.SetVoltage(1.7);  // 2.55 V at the sensor: 50 mV offset from VCC/2
  CHECK(acs.GetCurrent().value() == Catch::Approx(1.25));
  acs.CalibrateZero(10);
  CHECK(acs.GetZeroVoltage().value() == Catch::Approx(2.55));
  CHECK(acs.GetCurrent().value() == Catch::Approx(0.0).margin(1e-9));
  adc.SetVoltage((2.55 + 0.4) / 1.5);
  CHECK(acs.GetCurrent().value() == Catch::Approx(10.0));

  adc.SetVoltage(1.0);
  acs.CalibrateZero(0);  // at least one sample
  CHECK(acs.GetZeroVoltage().value() == Catch::Approx(1.5));
}

TEST_CASE("AnalogCurrentSensor general constructor and invalid values", "[analogcurrent]") {
  sim::AnalogInputSim adc{6};
  adc.SetVoltage(1.0);

  // negative sensitivity: a sensor wired the other way round
  AnalogCurrentSensor reversed{6, -0.1, 0.5};
  CHECK(reversed.GetCurrent().value() == Catch::Approx(-5.0));
  CHECK(reversed.GetTelemetryType() == "Current Sensor");

  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  {
    AnalogCurrentSensor zero{7, 0.0, 0.0};  // invalid sensitivity: reads 0 A
    sim::AnalogInputSim{7}.SetVoltage(2.0);
    CHECK(zero.GetCurrent().value() == 0.0);
  }
  {
    AnalogCurrentSensor nanSens{7, kNaN, 0.0};
    CHECK(nanSens.GetCurrent().value() == 0.0);
  }
  {
    // a divider below 1 (or NaN) is refused: ratio 1
    AnalogCurrentSensor badDivider{7, 0.2, 0.0, 0.5};
    CHECK(badDivider.GetVoltage().value() == Catch::Approx(2.0));
    CHECK(badDivider.GetCurrent().value() == Catch::Approx(10.0));
  }
  {
    AnalogCurrentSensor nanDivider{7, 0.2, kNaN, kNaN};  // NaN zero: 0 V
    CHECK(nanDivider.GetZeroVoltage().value() == 0.0);
    CHECK(nanDivider.GetCurrent().value() == Catch::Approx(10.0));
  }
}
