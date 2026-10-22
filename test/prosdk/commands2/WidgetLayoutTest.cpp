// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// Golden layouts of the Field2d, Mechanism2d and Scheduler widgets as prosdk publishes them to
// probot-core's dashboard store (fake core store): every topic (path, value type, type string,
// properties, flags, decoded value) and every struct schema, exactly. These are the layouts
// probot-core's Driver Station widgets decode (design/core-requests.md, "Widget layouts for
// Field2d, Mechanism2d, Scheduler"). Also the Driver Station write-back of the mutable topics
// (a Field2d pose drag, a Scheduler cancel) and what it does on the robot.
//
// In this target (not prosdk_dashboard_tests) because the CommandScheduler needs the HAL
// (HAL_Initialize in commands2/main.cpp).

#include <stdint.h>
#include <stdio.h>

#include <cstring>
#include <map>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "DashboardFixture.hpp"
#include "prosdk/commands2/CommandScheduler.hpp"
#include "prosdk/commands2/FunctionalCommand.hpp"
#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/smartdashboard/Field2d.hpp"
#include "prosdk/smartdashboard/Mechanism2d.hpp"
#include "prosdk/smartdashboard/MechanismLigament2d.hpp"
#include "prosdk/smartdashboard/MechanismRoot2d.hpp"
#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/tunables/Tunables.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/util/Color8Bit.hpp"

using dashboard_test::DashboardFixture;
namespace dash = prosdk::platform::dashboard;
using namespace prosdk::cmd;

namespace {

uint8_t VT(dash::ValueType vt) {
  return static_cast<uint8_t>(vt);
}

std::string Num(double v) {
  char buf[40];
  snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

const char* VtName(uint8_t vt) {
  static const char* names[] = {"?",     "boolean", "int",       "float",    "double",
                                "string", "raw",    "boolean[]", "int[]",    "float[]",
                                "double[]", "string[]"};
  return vt < 12 ? names[vt] : "?";
}

std::string FlagNames(uint8_t flags) {
  static const char* names[] = {"TELEMETRY", "TUNABLE",         "MUTABLE",
                                "ROBUST",    "KEEP_DUPLICATES", "PERSISTENT"};
  std::string out;
  for (int i = 0; i < 6; ++i) {
    if (flags & (1u << i)) {
      out += out.empty() ? "" : "|";
      out += names[i];
    }
  }
  return out.empty() ? "0" : out;
}

/** Decodes a value; int64 values found in ids are replaced by their symbolic name. */
std::string Decode(const fake::Topic& t, const std::map<int64_t, std::string>& ids) {
  if (!t.hasValue) {
    return "<no value>";
  }
  auto& v = t.value;
  auto f64 = [&](size_t off) {
    double d;
    std::memcpy(&d, v.data() + off, 8);
    return d;
  };
  auto i64 = [&](size_t off) {
    int64_t i;
    std::memcpy(&i, v.data() + off, 8);
    auto it = ids.find(i);
    return it == ids.end() ? std::to_string(i) : it->second;
  };
  std::string out;
  switch (static_cast<dash::ValueType>(t.valueType)) {
    case dash::ValueType::kBoolean:
      return v.size() == 1 ? (v[0] ? "true" : "false") : "<bad>";
    case dash::ValueType::kInt:
      return v.size() == 8 ? i64(0) : "<bad>";
    case dash::ValueType::kDouble:
      return v.size() == 8 ? Num(f64(0)) : "<bad>";
    case dash::ValueType::kString:
      return "\"" + std::string(v.begin(), v.end()) + "\"";
    case dash::ValueType::kIntArray:
      out = "[";
      for (size_t i = 0; i + 8 <= v.size(); i += 8) {
        out += (i ? ", " : "") + i64(i);
      }
      return out + "]";
    case dash::ValueType::kDoubleArray:
      out = "[";
      for (size_t i = 0; i + 8 <= v.size(); i += 8) {
        out += (i ? ", " : "") + Num(f64(i));
      }
      return out + "]";
    case dash::ValueType::kStringArray: {
      std::vector<std::string> strings;
      if (!dash::DecodeStringArray(v, strings)) {
        return "<bad>";
      }
      out = "[";
      for (size_t i = 0; i < strings.size(); ++i) {
        out += (i ? ", \"" : "\"") + strings[i] + "\"";
      }
      return out + "]";
    }
    case dash::ValueType::kRaw:
      // struct:Pose2d / struct:Pose2d[]: f64 x n, LE
      if (t.typeString.rfind("struct:Pose2d", 0) == 0 && v.size() % 24 == 0) {
        out = std::to_string(v.size()) + " bytes {";
        for (size_t i = 0; i < v.size(); i += 24) {
          out += (i ? ", " : "") + std::string("(") + Num(f64(i)) + ", " + Num(f64(i + 8)) +
                 ", " + Num(f64(i + 16)) + ")";
        }
        return out + "}";
      }
      [[fallthrough]];
    default:
      out = std::to_string(v.size()) + " bytes";
      return out;
  }
}

/** Every live topic under prefix, one line each. */
std::string DumpTopics(std::string_view prefix, const std::map<int64_t, std::string>& ids = {}) {
  std::string out;
  for (auto& t : fake::AllTopics()) {
    if (t.path.rfind(prefix, 0) != 0) {
      continue;
    }
    out += t.path + " | " + VtName(t.valueType) + " | " + t.typeString + " | " + t.properties +
           " | " + FlagNames(t.flags) + " | " + Decode(t, ids) + "\n";
  }
  return out;
}

std::string DumpSchemas() {
  std::string out;
  for (auto& name : fake::SchemaNames()) {
    auto s = fake::FindSchema(name);
    out += name + " | " + s->type + " | " + std::string(s->data.begin(), s->data.end()) + "\n";
  }
  return out;
}

std::vector<uint8_t> PosesBytes(const std::vector<std::vector<double>>& poses) {
  std::vector<uint8_t> out;
  for (auto& p : poses) {
    for (double d : p) {
      dash::Append<double>(out, d);
    }
  }
  return out;
}

std::vector<uint8_t> EncodeIntArray(const std::vector<int64_t>& values) {
  std::vector<uint8_t> out;
  for (int64_t v : values) {
    dash::Append<int64_t>(out, v);
  }
  return out;
}

int64_t IdOf(const Command& command) {
  return static_cast<int64_t>(reinterpret_cast<uintptr_t>(&command));
}

class WidgetFixture : public DashboardFixture {
 public:
  WidgetFixture() {
    auto& scheduler = CommandScheduler::GetInstance();
    scheduler.CancelAll();
    scheduler.Enable();
    scheduler.GetActiveButtonLoop()->Clear();
    scheduler.UnregisterAllSubsystems();
    prosdk::sim::DriverStationSim::SetDsAttached(true);
    prosdk::sim::DriverStationSim::SetEnabled(true);
    prosdk::sim::DriverStationSim::NotifyNewData();
  }

  ~WidgetFixture() {
    auto& scheduler = CommandScheduler::GetInstance();
    scheduler.CancelAll();
    scheduler.GetActiveButtonLoop()->Clear();
    scheduler.UnregisterAllSubsystems();
  }
};

FunctionalCommand Forever() {
  return FunctionalCommand{[] {}, [] {}, [](bool) {}, [] { return false; }};
}

}  // namespace

TEST_CASE_METHOD(WidgetFixture, "Widget layout: Field2d telemetry, tunable and pose drag",
                 "[prosdk][dashboard][widgets]") {
  using prosdk::math::Pose2d;
  using prosdk::math::Rotation2d;
  using namespace prosdk::units;

  prosdk::Field2d field;
  field.SetRobotPose(1.5_m, 2.25_m, Rotation2d{90_deg});
  field.GetObject("Targets")->SetPoses(
      {Pose2d{4_m, 0.5_m, Rotation2d{180_deg}}, Pose2d{6.25_m, 3_m, Rotation2d{-45_deg}}});

  prosdk::telemetry::Log("Field", field);
  REQUIRE(prosdk::tunables::Publish("Field", field));
  DashboardFixture::Cycle();

  auto telemetry = DumpTopics("/Telemetry/");
  auto tunables = DumpTopics("/Tunables/");
  auto schemas = DumpSchemas();

  CHECK(telemetry == R"(/Telemetry/Field/.type | string | string |  | TELEMETRY | "Field2d"
/Telemetry/Field/Robot | raw | struct:Pose2d[] |  | TELEMETRY | 24 bytes {(1.5, 2.25, 1.5707963267948966)}
/Telemetry/Field/Targets | raw | struct:Pose2d[] |  | TELEMETRY | 48 bytes {(4, 0.5, 3.1415926535897931), (6.25, 3, -0.78539816339744828)}
)");
  CHECK(tunables == R"(/Tunables/Field/.type | string | string | {"mutable":false} | TUNABLE | "Field2d"
/Tunables/Field/Robot | raw | struct:Pose2d[] | {"mutable":true} | TUNABLE|MUTABLE | 24 bytes {(1.5, 2.25, 1.5707963267948966)}
/Tunables/Field/Targets | raw | struct:Pose2d[] | {"mutable":true} | TUNABLE|MUTABLE | 48 bytes {(4, 0.5, 3.1415926535897931), (6.25, 3, -0.78539816339744828)}
)");
  CHECK(schemas == R"(struct:Pose2d | structschema | Translation2d translation;Rotation2d rotation
struct:Rotation2d | structschema | double value
struct:Translation2d | structschema | double x;double y
)");

  // Driver Station drags the robot: a write of the whole struct:Pose2d[] value
  auto robot = fake::Find("/Tunables/Field/Robot");
  REQUIRE(robot);
  fake::PushWrite("/Tunables/Field/Robot", robot->valueType, PosesBytes({{3, 1, 0.5}}));
  DashboardFixture::Cycle();
  CHECK(fake::Rejects().empty());
  auto pose = field.GetRobotPose();
  CHECK(pose.X().value() == 3);
  CHECK(pose.Y().value() == 1);
  CHECK(pose.Rotation().Radians().value() == 0.5);
  CHECK(fake::GetRaw("/Tunables/Field/Robot") == PosesBytes({{3, 1, 0.5}}));

  // the write replaces the whole array (here two poses -> one); an empty array clears it
  fake::PushWrite("/Tunables/Field/Targets", robot->valueType, PosesBytes({{1, 2, 0}}));
  DashboardFixture::Cycle();
  CHECK(field.GetObject("Targets")->GetPoses() == std::vector<Pose2d>{Pose2d{1_m, 2_m, 0_rad}});
  fake::PushWrite("/Tunables/Field/Targets", robot->valueType, {});
  DashboardFixture::Cycle();
  CHECK(field.GetObject("Targets")->GetPoses().empty());
  CHECK(fake::Rejects().empty());

  // a length that is not a multiple of 24 bytes is refused
  std::vector<uint8_t> bad(25, 0);
  fake::PushWrite("/Tunables/Field/Robot", robot->valueType, bad);
  DashboardFixture::Cycle();
  REQUIRE(fake::Rejects().size() == 1);
  CHECK(fake::Rejects()[0].first == "/Tunables/Field/Robot");
  CHECK(fake::Rejects()[0].second == static_cast<uint8_t>(dash::WriteStatus::kDecodeError));
  CHECK(field.GetRobotPose().X().value() == 3);

  prosdk::tunables::Remove("Field");
}

TEST_CASE_METHOD(WidgetFixture, "Widget layout: Mechanism2d telemetry",
                 "[prosdk][dashboard][widgets]") {
  using namespace prosdk::units;
  prosdk::Mechanism2d mech{3, 2, prosdk::util::Color8Bit{0, 0, 32}};
  auto shoulder = mech.GetRoot("shoulder", 1, 0.5);
  auto arm = shoulder->Append<prosdk::MechanismLigament2d>("arm", 1.25, 45_deg, 4.0,
                                                            prosdk::util::Color8Bit{255, 0, 0});
  arm->Append<prosdk::MechanismLigament2d>("wrist", 0.5, -30_deg, 2.5,
                                           prosdk::util::Color8Bit{0, 128, 255});
  auto base = mech.GetRoot("base", 2.5, 0);
  base->Append<prosdk::MechanismLigament2d>("post", 1.5, 90_deg);

  prosdk::telemetry::Log("Arm", mech);
  DashboardFixture::Cycle();

  auto telemetry = DumpTopics("/Telemetry/");
  CHECK(telemetry == R"(/Telemetry/Arm/.type | string | string |  | TELEMETRY | "Mechanism2d"
/Telemetry/Arm/backgroundColor | string | string |  | TELEMETRY | "#000020"
/Telemetry/Arm/base/position | double[] | double[] |  | TELEMETRY | [2.5, 0]
/Telemetry/Arm/base/post/.type | string | string |  | TELEMETRY | "line"
/Telemetry/Arm/base/post/angle | double | double |  | TELEMETRY | 90
/Telemetry/Arm/base/post/color | string | string |  | TELEMETRY | "#EB8934"
/Telemetry/Arm/base/post/length | double | double |  | TELEMETRY | 1.5
/Telemetry/Arm/base/post/weight | double | double |  | TELEMETRY | 6
/Telemetry/Arm/dims | double[] | double[] |  | TELEMETRY | [3, 2]
/Telemetry/Arm/shoulder/arm/.type | string | string |  | TELEMETRY | "line"
/Telemetry/Arm/shoulder/arm/angle | double | double |  | TELEMETRY | 45
/Telemetry/Arm/shoulder/arm/color | string | string |  | TELEMETRY | "#FF0000"
/Telemetry/Arm/shoulder/arm/length | double | double |  | TELEMETRY | 1.25
/Telemetry/Arm/shoulder/arm/weight | double | double |  | TELEMETRY | 4
/Telemetry/Arm/shoulder/arm/wrist/.type | string | string |  | TELEMETRY | "line"
/Telemetry/Arm/shoulder/arm/wrist/angle | double | double |  | TELEMETRY | -30
/Telemetry/Arm/shoulder/arm/wrist/color | string | string |  | TELEMETRY | "#0080FF"
/Telemetry/Arm/shoulder/arm/wrist/length | double | double |  | TELEMETRY | 0.5
/Telemetry/Arm/shoulder/arm/wrist/weight | double | double |  | TELEMETRY | 2.5
/Telemetry/Arm/shoulder/position | double[] | double[] |  | TELEMETRY | [1, 0.5]
)");
  // telemetry only: Mechanism2d is not a tunable, the Driver Station cannot change it
  CHECK(DumpTopics("/Tunables/") == "");
  CHECK(fake::SchemaNames().empty());

  // a later change shows up on the next log
  arm->SetAngle(60_deg);
  prosdk::telemetry::Log("Arm", mech);
  DashboardFixture::Cycle();
  CHECK(fake::GetDouble("/Telemetry/Arm/shoulder/arm/angle") == 60.0);
}

TEST_CASE_METHOD(WidgetFixture, "Widget layout: Scheduler telemetry, tunable and cancel",
                 "[prosdk][dashboard][widgets]") {
  auto& scheduler = CommandScheduler::GetInstance();
  auto drive = Forever();
  drive.SetName("Drive");
  auto intake = Forever();
  intake.SetName("Intake");
  scheduler.Schedule(&drive);
  scheduler.Schedule(&intake);
  scheduler.Run();

  prosdk::telemetry::Log("Scheduler", scheduler);
  REQUIRE(prosdk::tunables::Publish("Scheduler", scheduler));
  DashboardFixture::Cycle();

  std::map<int64_t, std::string> ids{{IdOf(drive), "<id Drive>"}, {IdOf(intake), "<id Intake>"}};
  auto telemetry = DumpTopics("/Telemetry/", ids);
  auto tunables = DumpTopics("/Tunables/", ids);
  CHECK(telemetry == R"(/Telemetry/Scheduler/.type | string | string |  | TELEMETRY | "Scheduler"
/Telemetry/Scheduler/Ids | int[] | int[] |  | TELEMETRY | [<id Drive>, <id Intake>]
/Telemetry/Scheduler/Names | string[] | string[] |  | TELEMETRY | ["Drive", "Intake"]
)");
  CHECK(tunables == R"(/Tunables/Scheduler/.type | string | string | {"mutable":false} | TUNABLE | "Scheduler"
/Tunables/Scheduler/Cancel | int[] | int[] | {"mutable":true,"robust":true} | TUNABLE|MUTABLE|ROBUST | []
/Tunables/Scheduler/Ids | int[] | int[] | {"mutable":false} | TUNABLE | [<id Drive>, <id Intake>]
/Tunables/Scheduler/Names | string[] | string[] | {"mutable":false} | TUNABLE | ["Drive", "Intake"]
)");
  CHECK(fake::SchemaNames().empty());

  // Driver Station presses cancel on Intake: Cancel = [id] (unknown ids are ignored); the robot
  // cancels the ids still scheduled and clears Cancel
  auto cancel = fake::Find("/Tunables/Scheduler/Cancel");
  REQUIRE(cancel);
  fake::PushWrite("/Tunables/Scheduler/Cancel", VT(dash::ValueType::kIntArray),
                  EncodeIntArray({IdOf(intake), 12345}));
  DashboardFixture::Cycle();
  CHECK(fake::Rejects().empty());
  CHECK_FALSE(scheduler.IsScheduled(&intake));
  CHECK(scheduler.IsScheduled(&drive));
  DashboardFixture::Cycle();
  CHECK(fake::GetIntArray("/Tunables/Scheduler/Cancel") == std::vector<int64_t>{});
  CHECK(fake::GetStringArray("/Tunables/Scheduler/Names") == std::vector<std::string>{"Drive"});
  CHECK(fake::GetIntArray("/Tunables/Scheduler/Ids") == std::vector<int64_t>{IdOf(drive)});

  // read-only children refuse writes
  fake::PushWrite("/Tunables/Scheduler/Names", VT(dash::ValueType::kStringArray),
                  fake::EncodeStringArray(std::vector<std::string>{"x"}));
  DashboardFixture::Cycle();
  REQUIRE(fake::Rejects().size() == 1);
  CHECK(fake::Rejects()[0].first == "/Tunables/Scheduler/Names");
  CHECK(fake::Rejects()[0].second == static_cast<uint8_t>(dash::WriteStatus::kImmutable));

  prosdk::tunables::Remove("Scheduler");
}
