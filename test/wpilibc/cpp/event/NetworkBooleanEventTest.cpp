// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/event/NetworkBooleanEvent.hpp"

#include <memory>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "prosdk/driverstation/internal/DriverStationBackend.hpp"
#include "prosdk/event/EventLoop.hpp"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/tunables/MockTunableBackend.hpp"
#include "prosdk/tunables/Tunable.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"
#include "prosdk/tunables/Tunables.hpp"

using namespace prosdk;

// prosdk: NetworkTables is not ported; the topic is a tunable on a mock tunable
// backend, and "connected" is the simulated Driver Station being attached.
class NetworkBooleanEventTest {
 public:
  NetworkBooleanEventTest() {
    prosdk::tunables::TunableRegistry::Reset();
    m_backend = std::make_shared<prosdk::tunables::MockTunableBackend>();
    prosdk::tunables::TunableRegistry::RegisterBackend("", m_backend);
    SetDsAttached(true);
  }

  ~NetworkBooleanEventTest() {
    prosdk::tunables::TunableRegistry::Reset();
    prosdk::sim::DriverStationSim::ResetData();
  }

  void Set(bool value) {
    m_backend->SetBool("/TestTable/Test", value);
    prosdk::tunables::TunableRegistry::Update();
  }

  static void SetDsAttached(bool attached) {
    // new DS data attaches the simulated DS, so a detach sends none (as
    // DriverStationSimTest and OpModeLifecycleTest do)
    prosdk::sim::DriverStationSim::SetDsAttached(attached);
    if (attached) {
      prosdk::sim::DriverStationSim::NotifyNewData();
    }
    prosdk::internal::DriverStationBackend::RefreshData();
  }

  std::shared_ptr<prosdk::tunables::MockTunableBackend> m_backend;
};

TEST_CASE_METHOD(NetworkBooleanEventTest, "NetworkBooleanEventTest Set",
                 "[wpilibc][event]") {
  EventLoop loop;
  int counter = 0;

  NetworkBooleanEvent(&loop, "TestTable", "Test").IfHigh([&] {
    ++counter;
  });
  Set(false);
  loop.Poll();
  CHECK(0 == counter);
  Set(true);
  loop.Poll();
  CHECK(1 == counter);
  Set(false);
  loop.Poll();
  CHECK(1 == counter);
}

// prosdk: the cases below cover the tunable-specific behavior

TEST_CASE_METHOD(NetworkBooleanEventTest,
                 "NetworkBooleanEventTest PublishesFalse", "[wpilibc][event]") {
  EventLoop loop;
  NetworkBooleanEvent event{&loop, "TestTable/Test"};
  prosdk::tunables::TunableRegistry::Update();
  CHECK_FALSE(m_backend->GetBool("/TestTable/Test"));
  CHECK_FALSE(event.GetAsBoolean());
}

TEST_CASE_METHOD(NetworkBooleanEventTest,
                 "NetworkBooleanEventTest InactiveWithoutDriverStation",
                 "[wpilibc][event]") {
  EventLoop loop;
  int counter = 0;
  NetworkBooleanEvent(&loop, "TestTable", "Test").IfHigh([&] {
    ++counter;
  });
  SetDsAttached(false);
  Set(true);
  loop.Poll();
  CHECK(0 == counter);
  SetDsAttached(true);
  loop.Poll();
  CHECK(1 == counter);
}

TEST_CASE_METHOD(NetworkBooleanEventTest,
                 "NetworkBooleanEventTest SameNameSharesTunable",
                 "[wpilibc][event]") {
  EventLoop loop;
  int first = 0;
  int second = 0;
  NetworkBooleanEvent(&loop, "TestTable", "Test").IfHigh([&] { ++first; });
  NetworkBooleanEvent(&loop, prosdk::tunables::GetTable("TestTable"), "Test")
      .IfHigh([&] { ++second; });
  Set(true);
  loop.Poll();
  CHECK(1 == first);
  CHECK(1 == second);
}

TEST_CASE_METHOD(NetworkBooleanEventTest, "NetworkBooleanEventTest Tunable",
                 "[wpilibc][event]") {
  EventLoop loop;
  int counter = 0;
  NetworkBooleanEvent(&loop, prosdk::tunables::Add<bool>("TestTable/Test"))
      .IfHigh([&] { ++counter; });
  Set(true);
  loop.Poll();
  CHECK(1 == counter);
}
