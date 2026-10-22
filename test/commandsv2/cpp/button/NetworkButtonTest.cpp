// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/button/NetworkButton.hpp"

#include <memory>

#include <catch2/catch_test_macros.hpp>

#include "../CommandTestBase.hpp"
#include "prosdk/commands2/CommandScheduler.hpp"
#include "prosdk/commands2/WaitUntilCommand.hpp"
#include "prosdk/driverstation/internal/DriverStationBackend.hpp"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/tunables/MockTunableBackend.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"

using namespace prosdk::cmd;

// prosdk: NetworkButton is a boolean tunable (ntcore is not ported); the
// dashboard is the mock tunable backend instead of a local NT instance.
class NetworkButtonTest : public CommandTestBase {
 public:
  NetworkButtonTest() {
    prosdk::tunables::TunableRegistry::Reset();
    backend = std::make_shared<prosdk::tunables::MockTunableBackend>();
    prosdk::tunables::TunableRegistry::RegisterBackend("", backend);
  }

  ~NetworkButtonTest() override { prosdk::tunables::TunableRegistry::Reset(); }

  std::shared_ptr<prosdk::tunables::MockTunableBackend> backend;
};

TEST_CASE_METHOD(NetworkButtonTest, "NetworkButtonTest SetNetworkButton",
                 "[commandsv2][command]") {
  auto& scheduler = CommandScheduler::GetInstance();
  bool finished = false;

  WaitUntilCommand command([&finished] { return finished; });

  NetworkButton("TestTable", "Test").OnTrue(&command);
  scheduler.Run();
  CHECK_FALSE(scheduler.IsScheduled(&command));
  backend->SetBool("/TestTable/Test", true);
  prosdk::tunables::TunableRegistry::Update();
  scheduler.Run();
  CHECK(scheduler.IsScheduled(&command));
  finished = true;
  scheduler.Run();
  CHECK_FALSE(scheduler.IsScheduled(&command));
}

// prosdk: the cases below test the tunables re-implementation.
TEST_CASE_METHOD(NetworkButtonTest, "NetworkButtonTest SharedPathAndDisconnect",
                 "[commandsv2][command]") {
  auto& scheduler = CommandScheduler::GetInstance();
  NetworkButton a{"/Buttons/Shoot"};
  NetworkButton b{"Buttons", "Shoot"};
  CHECK(backend->GetUid("/Buttons/Shoot"));
  CHECK_FALSE(a.Get());
  CHECK_FALSE(b.Get());

  backend->SetBool("/Buttons/Shoot", true);
  prosdk::tunables::TunableRegistry::Update();
  CHECK(a.Get());
  CHECK(b.Get());

  // upstream: false while NetworkTables is not connected; here while no
  // Driver Station is attached (NotifyNewData marks the DS attached again)
  prosdk::sim::DriverStationSim::SetDsAttached(false);
  prosdk::internal::DriverStationBackend::RefreshData();
  CHECK_FALSE(a.Get());
  prosdk::sim::DriverStationSim::NotifyNewData();
  CHECK(a.Get());
  scheduler.Run();
}

TEST_CASE_METHOD(NetworkButtonTest, "NetworkButtonTest RemovedWithLastButton",
                 "[commandsv2][command]") {
  {
    NetworkButton button{"Buttons/Temp"};
    CHECK(backend->GetUid("/Buttons/Temp"));
  }
  CHECK_FALSE(backend->GetUid("/Buttons/Temp"));
}
