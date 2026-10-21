// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/util/UsageReporting.h"

#include <string>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/util/UsageReporting.hpp"
#include "prosdk/util/string.hpp"

namespace {

std::string resource;
std::string data;

void MockReportUsage(const WPI_String* resourceStr, const WPI_String* dataStr) {
  resource = prosdk::util::to_string_view(resourceStr);
  data = prosdk::util::to_string_view(dataStr);
}

class ResetReportUsageImpl {
 public:
  ~ResetReportUsageImpl() { prosdk::util::SetReportUsageImpl(nullptr); }
};

}  // namespace

TEST_CASE("UsageReportingTest DefaultNoOp", "[wpiutil]") {
  prosdk::util::ReportUsage("Resource", "Data");
  WPI_ReportUsage(nullptr, nullptr);
}

TEST_CASE("UsageReportingTest CppReportUsageCallsImpl", "[wpiutil]") {
  ResetReportUsageImpl reset;
  prosdk::util::SetReportUsageImpl(MockReportUsage);

  prosdk::util::ReportUsage("Resource", "Data");

  CHECK("Resource" == resource);
  CHECK("Data" == data);
}

TEST_CASE("UsageReportingTest CppReportUsageInstanceFormatsResource",
          "[wpiutil]") {
  ResetReportUsageImpl reset;
  prosdk::util::SetReportUsageImpl(MockReportUsage);

  prosdk::util::ReportUsage("Resource", 4, "Data");

  CHECK("Resource[4]" == resource);
  CHECK("Data" == data);
}

TEST_CASE("UsageReportingTest CReportUsageInstanceFormatsResource",
          "[wpiutil]") {
  ResetReportUsageImpl reset;
  WPI_SetReportUsageImpl(MockReportUsage);

  WPI_String resourceStr = prosdk::util::make_string("Resource");
  WPI_String dataStr = prosdk::util::make_string("Data");
  WPI_ReportUsageInstance(&resourceStr, 5, &dataStr);

  CHECK("Resource[5]" == resource);
  CHECK("Data" == data);
}
