// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// (b) C++ exceptions across the precompiled archive (thrown in the archive and caught in the
// sketch, thrown in the sketch and unwound through archive frames), std::format (floats,
// units, chrono) and the console paths (prosdk::cxx::println to stdout, the platform console to
// the Driver Station console + UART).

#include <chrono>
#include <exception>
#include <format>
#include <stdexcept>
#include <string>
#include <vector>

#include <prosdk/cxx/print.hpp>
#include <prosdk/hardware/discrete/DigitalInput.hpp>
#include <prosdk/math/geometry/Pose2d.hpp>
#include <prosdk/math/trajectory/DrivetrainSplineTrajectoryGenerator.hpp>
#include <prosdk/math/trajectory/TrajectoryConfig.hpp>
#include <prosdk/math/trajectory/constraint/TrajectoryConstraint.hpp>
#include <prosdk/platform/ConsoleFormat.hpp>
#include <prosdk/system/Errors.hpp>
#include <prosdk/units/acceleration.hpp>
#include <prosdk/units/angle.hpp>
#include <prosdk/units/length.hpp>
#include <prosdk/units/velocity.hpp>

#include "FakeCore.hpp"
#include "Harness.hpp"

namespace qemu {

namespace pm = prosdk::math;

namespace {

// A user constraint called from the archive's trajectory parameterizer.
class ThrowingConstraint : public pm::TrajectoryConstraint {
 public:
  prosdk::units::meters_per_second_t MaxVelocity(
      const pm::Pose2d&, prosdk::units::curvature_t,
      prosdk::units::meters_per_second_t) const override {
    throw std::domain_error("qemu: thrown in the sketch, unwound through the archive");
  }
  MinMax MinMaxAcceleration(const pm::Pose2d&, prosdk::units::curvature_t,
                            prosdk::units::meters_per_second_t) const override {
    return {};
  }
};

// A constraint that stops the robot: the archive's parameterizer throws std::runtime_error.
class StopConstraint : public pm::TrajectoryConstraint {
 public:
  prosdk::units::meters_per_second_t MaxVelocity(
      const pm::Pose2d&, prosdk::units::curvature_t,
      prosdk::units::meters_per_second_t) const override {
    return 0_mps;
  }
  MinMax MinMaxAcceleration(const pm::Pose2d&, prosdk::units::curvature_t,
                            prosdk::units::meters_per_second_t) const override {
    return {0_mps_sq, 0_mps_sq};
  }
};

template <class Constraint>
void Generate() {
  pm::TrajectoryConfig config{1_mps, 1_mps_sq};
  config.AddConstraint(Constraint{});
  std::vector<pm::Translation2d> none;
  (void)pm::DrivetrainSplineTrajectoryGenerator::Generate(pm::Pose2d{}, none,
                                                          pm::Pose2d{1_m, 0_m, 0_deg}, config);
}

}  // namespace

void CaseExceptions(Ctx& ctx) {
  // 1. sketch -> archive -> sketch
  bool caught = false;
  try {
    Generate<ThrowingConstraint>();
  } catch (const std::domain_error& e) {
    caught = std::string_view{e.what()}.find("unwound through the archive") != std::string::npos;
  }
  QCHECK_MSG(ctx, caught, "std::domain_error through DrivetrainSplineTrajectoryParameterizer");

  // 2. thrown by the archive (std::runtime_error with a std::format message)
  std::string what;
  try {
    Generate<StopConstraint>();
  } catch (const std::runtime_error& e) {
    what = e.what();
  }
  QCHECK_MSG(ctx, what.find("time parameterization") != std::string::npos, "what='{}'", what);

  // 3. wpilibc error path: HAL status -> prosdk::RuntimeError (GPIO 22 does not exist on the
  //    ESP32-S3; the HAL refuses it before touching a peripheral). Also initializes the HAL.
  std::string halWhat;
  int32_t code = 0;
  try {
    prosdk::DigitalInput input{22};
  } catch (const prosdk::RuntimeError& e) {
    halWhat = e.what();
    code = e.code();
  }
  QCHECK_MSG(ctx, !halWhat.empty() && code < 0, "code={} what='{}'", code, halWhat);
  ctx.Note("hal_error_code={}", code);

  // 4. nested rethrow and std::exception_ptr across tasks of the same image
  std::exception_ptr ptr;
  try {
    try {
      throw std::out_of_range("inner");
    } catch (...) {
      std::throw_with_nested(std::logic_error("outer"));
    }
  } catch (...) {
    ptr = std::current_exception();
  }
  // Caught as std::nested_exception: under -fno-rtti, libstdc++'s std::rethrow_if_nested is a
  // no-op for a static type that does not derive from nested_exception (it cannot dynamic_cast),
  // so rethrow_if_nested(const std::logic_error&) would silently do nothing. Matching a handler
  // uses the exception tables, which -fno-rtti keeps.
  bool outerOk = false;
  bool nestedOk = false;
  try {
    std::rethrow_exception(ptr);
  } catch (const std::nested_exception& ne) {
    try {
      std::rethrow_exception(ptr);
    } catch (const std::logic_error& e) {
      outerOk = std::string_view{e.what()} == "outer";
    }
    try {
      ne.rethrow_nested();
    } catch (const std::out_of_range& inner) {
      nestedOk = std::string_view{inner.what()} == "inner";
    }
  }
  QCHECK(ctx, outerOk);
  QCHECK(ctx, nestedOk);

  // 5. std::format
  QCHECK(ctx, std::format("{:.3f}|{:>5}|{:#x}|{}", 3.14159265, 42, 255, true) ==
                  "3.142|   42|0xff|true");
  QCHECK(ctx, std::format("{:e}", 1.5e-7) == "1.500000e-07");
  std::string unitText = std::format("{}", 1.5_m);
  QCHECK_MSG(ctx, unitText == "1.5 m", "units '{}'", unitText);
  std::string chronoText = std::format("{:%H:%M:%S}", std::chrono::seconds{3725});
  QCHECK_MSG(ctx, chronoText == "01:02:05", "chrono '{}'", chronoText);

  // 6. console: prosdk::cxx::println (stdout -> UART0) and the platform console
  //    (ConsoleWrite -> fake core's Driver Station console and stderr -> UART0)
  prosdk::cxx::println("QEMU-PRINT println {} {:.2f}", "ok", 2.5);
  prosdk::platform::ConsolePrint("QEMU-PRINT console {}\n", 7);
  QCHECK(ctx, fakecore::Console().find("QEMU-PRINT console 7") != std::string::npos);
}

}  // namespace qemu
