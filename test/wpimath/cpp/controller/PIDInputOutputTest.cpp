// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <memory>
#include <string_view>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/controller/PIDController.hpp"
#include "prosdk/telemetry/MockTelemetryBackend.hpp"
#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"
#include "prosdk/units/time.hpp"

namespace {
class PIDTelemetryTest {
 public:
  PIDTelemetryTest() {
    prosdk::telemetry::TelemetryRegistry::Reset();
    prosdk::telemetry::TelemetryRegistry::RegisterBackend("", backend);
  }

  ~PIDTelemetryTest() { prosdk::telemetry::TelemetryRegistry::Reset(); }

  template <typename T>
  T Last(std::string_view path) {
    auto value = backend->GetLastValue<T>(path);
    REQUIRE(value);
    return *value;
  }

  std::shared_ptr<prosdk::telemetry::MockTelemetryBackend> backend =
      std::make_shared<prosdk::telemetry::MockTelemetryBackend>();
};

static_assert([] {
  prosdk::math::PIDController controller{1.0, 2.0, 3.0};
  if (controller.GetP() != 1.0 || controller.GetI() != 2.0 ||
      controller.GetD() != 3.0) {
    return false;
  }
  controller.SetPID(4.0, 5.0, 6.0);
  controller.SetP(7.0);
  controller.SetI(8.0);
  controller.SetD(9.0);
  controller.SetIZone(10.0);
  return controller.GetP() == 7.0 && controller.GetI() == 8.0 &&
         controller.GetD() == 9.0 && controller.GetIZone() == 10.0;
}());
}  // namespace

TEST_CASE_METHOD(PIDTelemetryTest, "PIDInputOutputTest LogsTelemetry",
                 "[wpimath]") {
  prosdk::math::PIDController controller{0.5, 0.1, 0.01};
  controller.SetIZone(5.0);
  controller.Calculate(7.0, 10.0);

  prosdk::telemetry::Log("pid", controller);

  CHECK(Last<prosdk::telemetry::MockTelemetryBackend::LogStringValue>("/pid/.type")
            .value == "PIDController");
  CHECK(Last<double>("/pid/p") == 0.5);
  CHECK(Last<double>("/pid/i") == 0.1);
  CHECK(Last<double>("/pid/d") == 0.01);
  CHECK(Last<double>("/pid/izone") == 5.0);
  CHECK(Last<double>("/pid/setpoint") == 10.0);
  CHECK(Last<double>("/pid/measurement") == 7.0);
  CHECK(Last<double>("/pid/error") == 3.0);
}

TEST_CASE("PIDInputOutputTest ContinuousInput", "[wpimath]") {
  prosdk::math::PIDController controller{0.0, 0.0, 0.0};

  controller.SetP(1);
  controller.EnableContinuousInput(-180, 180);
  CHECK_DOUBLE_EQ(controller.Calculate(-179, 179), -2);

  controller.EnableContinuousInput(0, 360);
  CHECK_DOUBLE_EQ(controller.Calculate(1, 359), -2);
}

TEST_CASE("PIDInputOutputTest ProportionalGainOutput", "[wpimath]") {
  prosdk::math::PIDController controller{0.0, 0.0, 0.0};

  controller.SetP(4);

  CHECK_DOUBLE_EQ(-0.1, controller.Calculate(0.025, 0));
}

TEST_CASE("PIDInputOutputTest IntegralGainOutput", "[wpimath]") {
  prosdk::math::PIDController controller{0.0, 0.0, 0.0};

  controller.SetI(4);

  double out = 0;

  for (int i = 0; i < 5; i++) {
    out = controller.Calculate(0.025, 0);
  }

  CHECK_DOUBLE_EQ(-0.5 * controller.GetPeriod().value(), out);
}

TEST_CASE("PIDInputOutputTest DerivativeGainOutput", "[wpimath]") {
  prosdk::math::PIDController controller{0.0, 0.0, 0.0};

  controller.SetD(4);

  controller.Calculate(0, 0);

  CHECK_DOUBLE_EQ(-10_ms / controller.GetPeriod(),
                  controller.Calculate(0.0025, 0));
}

TEST_CASE("PIDInputOutputTest IZoneNoOutput", "[wpimath]") {
  prosdk::math::PIDController controller{0.0, 0.0, 0.0};

  controller.SetI(1);
  controller.SetIZone(1);

  double out = controller.Calculate(2, 0);

  CHECK_DOUBLE_EQ(0, out);
}

TEST_CASE("PIDInputOutputTest IZoneOutput", "[wpimath]") {
  prosdk::math::PIDController controller{0.0, 0.0, 0.0};

  controller.SetI(1);
  controller.SetIZone(1);

  double out = controller.Calculate(1, 0);

  CHECK_DOUBLE_EQ(-1 * controller.GetPeriod().value(), out);
}
