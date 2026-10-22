// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <memory>

#include "CommandTestBase.hpp"
#include "prosdk/telemetry/MockTelemetryBackend.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"

using namespace prosdk::cmd;

class CommandTelemetryTest : public CommandTestBase {
 protected:
  CommandTelemetryTest() {
    prosdk::telemetry::TelemetryRegistry::Reset();
    m_backend = std::make_shared<prosdk::telemetry::MockTelemetryBackend>();
    prosdk::telemetry::TelemetryRegistry::RegisterBackend("", m_backend);
  }

  ~CommandTelemetryTest() override {
    prosdk::telemetry::TelemetryRegistry::Reset();
  }

  std::shared_ptr<prosdk::telemetry::MockTelemetryBackend> m_backend;
};

TEST_CASE_METHOD(CommandTelemetryTest,
                 "CommandTelemetryTest CommandLogsMetadataWithoutLeadingDots",
                 "[commandsv2][command]") {
  MockCommand command;
  command.SetName("renamed");

  command.LogTo(prosdk::telemetry::TelemetryRegistry::GetTable("command"));

  auto name =
      m_backend
          ->GetLastValue<prosdk::telemetry::MockTelemetryBackend::LogStringValue>(
              "/command/name");
  REQUIRE(name);
  CHECK(name->value == "renamed");
  CHECK(m_backend->GetLastAction("/command/.name") == nullptr);

  auto isParented = m_backend->GetLastValue<bool>("/command/isParented");
  REQUIRE(isParented);
  CHECK_FALSE(*isParented);
  CHECK(m_backend->GetLastAction("/command/.isParented") == nullptr);
}
