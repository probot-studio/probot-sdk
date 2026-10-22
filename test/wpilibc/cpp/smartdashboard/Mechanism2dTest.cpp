// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/smartdashboard/Mechanism2d.hpp"

#include <memory>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/smartdashboard/MechanismLigament2d.hpp"
#include "prosdk/telemetry/MockTelemetryBackend.hpp"
#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/util/Color8Bit.hpp"

struct Mechanism2dTest {
  Mechanism2dTest() {
    prosdk::telemetry::TelemetryRegistry::Reset();
    prosdk::telemetry::TelemetryRegistry::RegisterBackend("", mock);
  }

  ~Mechanism2dTest() { prosdk::telemetry::TelemetryRegistry::Reset(); }

  std::shared_ptr<prosdk::telemetry::MockTelemetryBackend> mock =
      std::make_shared<prosdk::telemetry::MockTelemetryBackend>();
};

TEST_CASE_METHOD(Mechanism2dTest, "Mechanism2dTest Canvas",
                 "[wpilibc][smartdashboard]") {
  prosdk::Mechanism2d mechanism{5, 10};

  prosdk::telemetry::Log("mechanism", mechanism);
  {
    auto actions = mock->GetActions();
    REQUIRE(actions.size() == 3u);

    auto dims = mock->GetLastValue<std::vector<double>>("/mechanism/dims");
    REQUIRE(dims);
    REQUIRE(2u == dims->size());
    CHECK(5.0 == (*dims)[0]);
    CHECK(10.0 == (*dims)[1]);

    auto color = mock->GetLastValue<
        prosdk::telemetry::MockTelemetryBackend::LogStringValue>(
        "/mechanism/backgroundColor");
    REQUIRE(color);
    CHECK("#000020" == color->value);
    mock->Clear();
  }

  mechanism.SetBackgroundColor({255, 255, 255});
  prosdk::telemetry::Log("mechanism", mechanism);
  {
    auto color = mock->GetLastValue<
        prosdk::telemetry::MockTelemetryBackend::LogStringValue>(
        "/mechanism/backgroundColor");
    REQUIRE(color);
    CHECK("#FFFFFF" == color->value);
  }
}

TEST_CASE_METHOD(Mechanism2dTest, "Mechanism2dTest Root",
                 "[wpilibc][smartdashboard]") {
  prosdk::Mechanism2d mechanism{5, 10};
  prosdk::MechanismRoot2d* root = mechanism.GetRoot("root", 1, 2);
  prosdk::telemetry::Log("mechanism", mechanism);
  {
    auto pos =
        mock->GetLastValue<std::vector<double>>("/mechanism/root/position");
    REQUIRE(pos);
    REQUIRE(2u == pos->size());
    CHECK(1.0 == (*pos)[0]);
    CHECK(2.0 == (*pos)[1]);
    mock->Clear();
  }
  root->SetPosition(2, 4);
  prosdk::telemetry::Log("mechanism", mechanism);
  {
    auto pos =
        mock->GetLastValue<std::vector<double>>("/mechanism/root/position");
    REQUIRE(pos);
    REQUIRE(2u == pos->size());
    CHECK(2.0 == (*pos)[0]);
    CHECK(4.0 == (*pos)[1]);
  }
}

TEST_CASE_METHOD(Mechanism2dTest, "Mechanism2dTest Ligament",
                 "[wpilibc][smartdashboard]") {
  prosdk::Mechanism2d mechanism{5, 10};
  prosdk::MechanismRoot2d* root = mechanism.GetRoot("root", 1, 2);
  prosdk::MechanismLigament2d* ligament = root->Append<prosdk::MechanismLigament2d>(
      "ligament", 3, prosdk::units::degree_t{90}, 1,
      prosdk::util::Color8Bit{255, 255, 255});
  prosdk::telemetry::Log("mechanism", mechanism);
  {
    auto angle = mock->GetLastValue<double>("/mechanism/root/ligament/angle");
    REQUIRE(angle);
    CHECK(ligament->GetAngle() == *angle);
    auto color = mock->GetLastValue<
        prosdk::telemetry::MockTelemetryBackend::LogStringValue>(
        "/mechanism/root/ligament/color");
    REQUIRE(color);
    CHECK(ligament->GetColor().HexString() == color->value);
    auto length = mock->GetLastValue<double>("/mechanism/root/ligament/length");
    REQUIRE(length);
    CHECK(ligament->GetLength() == *length);
    auto weight = mock->GetLastValue<double>("/mechanism/root/ligament/weight");
    REQUIRE(weight);
    CHECK(ligament->GetLineWeight() == *weight);
    mock->Clear();
  }

  ligament->SetAngle(prosdk::units::degree_t{45});
  ligament->SetColor({0, 0, 0});
  ligament->SetLength(2);
  ligament->SetLineWeight(4);
  prosdk::telemetry::Log("mechanism", mechanism);
  {
    auto angle = mock->GetLastValue<double>("/mechanism/root/ligament/angle");
    REQUIRE(angle);
    CHECK(ligament->GetAngle() == *angle);
    auto color = mock->GetLastValue<
        prosdk::telemetry::MockTelemetryBackend::LogStringValue>(
        "/mechanism/root/ligament/color");
    REQUIRE(color);
    CHECK(ligament->GetColor().HexString() == color->value);
    auto length = mock->GetLastValue<double>("/mechanism/root/ligament/length");
    REQUIRE(length);
    CHECK(ligament->GetLength() == *length);
    auto weight = mock->GetLastValue<double>("/mechanism/root/ligament/weight");
    REQUIRE(weight);
    CHECK(ligament->GetLineWeight() == *weight);
  }
}
