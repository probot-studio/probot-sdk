// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// The LittleFS auto-mount decision behind prosdk::platform::GetLogDirectory()
// (prosdk/platform/StorageAutoMount.hpp) against a fake partition: an existing mount is used, the
// opt-out, a sketch with Arduino's LittleFS library and a program that links SPIFFS are left
// alone, only a blank partition is formatted, and a partition holding other data is never
// formatted.

#include <string>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/platform/Storage.hpp"
#include "prosdk/platform/StorageAutoMount.hpp"

namespace {

using prosdk::platform::detail::AutoMountLittleFs;
using prosdk::platform::detail::LittleFsAutoMount;
using prosdk::platform::detail::LittleFsMountStatus;

enum class Content { kBlank, kLittleFs, kOther };

class FakeDevice final : public prosdk::platform::detail::LittleFsDevice {
 public:
  bool mounted = false;
  bool arduinoLibrary = false;
  bool spiffs = false;
  bool partition = true;
  Content content = Content::kBlank;
  bool formatFails = false;
  bool mountError = false;  // e.g. out of memory, or the partition mounted elsewhere

  int blankChecks = 0;
  int formats = 0;
  int mounts = 0;

  bool IsMounted() override { return mounted; }
  bool ArduinoLibraryLinked() override { return arduinoLibrary; }
  bool SpiffsLinked() override { return spiffs; }
  bool HasPartition() override { return partition; }
  bool IsBlank() override {
    ++blankChecks;
    return content == Content::kBlank;
  }
  bool Format() override {
    ++formats;
    if (formatFails) {
      return false;
    }
    content = Content::kLittleFs;
    return true;
  }
  LittleFsMountStatus Mount() override {
    ++mounts;
    if (mountError) {
      return LittleFsMountStatus::kError;
    }
    if (content != Content::kLittleFs) {
      return LittleFsMountStatus::kNoFileSystem;
    }
    mounted = true;
    return LittleFsMountStatus::kOk;
  }
};

}  // namespace

TEST_CASE("LittleFS auto-mount uses an existing mount", "[prosdk][platform][storage]") {
  for (bool enabled : {true, false}) {
    FakeDevice dev;
    dev.mounted = true;
    dev.content = Content::kOther;
    CHECK(AutoMountLittleFs(dev, enabled) == LittleFsAutoMount::kAlreadyMounted);
    CHECK(dev.blankChecks == 0);
    CHECK(dev.formats == 0);
    CHECK(dev.mounts == 0);
  }
}

TEST_CASE("LittleFS auto-mount opt-out touches nothing", "[prosdk][platform][storage]") {
  FakeDevice dev;
  CHECK(AutoMountLittleFs(dev, false) == LittleFsAutoMount::kDisabled);
  CHECK(dev.blankChecks == 0);
  CHECK(dev.formats == 0);
  CHECK(dev.mounts == 0);
  CHECK_FALSE(dev.mounted);
}

TEST_CASE("LittleFS auto-mount leaves a sketch with the Arduino library alone",
          "[prosdk][platform][storage]") {
  FakeDevice dev;
  dev.arduinoLibrary = true;
  CHECK(AutoMountLittleFs(dev, true) == LittleFsAutoMount::kArduinoLibrary);
  CHECK(dev.formats == 0);
  CHECK(dev.mounts == 0);
}

TEST_CASE("LittleFS auto-mount leaves a program that links SPIFFS alone",
          "[prosdk][platform][storage]") {
  SECTION("blank partition: not formatted (SPIFFS.begin(true) would format it again)") {
    FakeDevice dev;
    dev.spiffs = true;
    CHECK(AutoMountLittleFs(dev, true) == LittleFsAutoMount::kSpiffs);
    CHECK(dev.blankChecks == 0);
    CHECK(dev.formats == 0);
    CHECK(dev.mounts == 0);
    CHECK(dev.content == Content::kBlank);
  }
  SECTION("an existing LittleFS: not mounted either") {
    FakeDevice dev;
    dev.spiffs = true;
    dev.content = Content::kLittleFs;
    CHECK(AutoMountLittleFs(dev, true) == LittleFsAutoMount::kSpiffs);
    CHECK(dev.mounts == 0);
    CHECK_FALSE(dev.mounted);
  }
  SECTION("a mount the sketch made, the opt-out and the Arduino LittleFS library come first") {
    FakeDevice dev;
    dev.spiffs = true;
    dev.mounted = true;
    CHECK(AutoMountLittleFs(dev, true) == LittleFsAutoMount::kAlreadyMounted);
    dev.mounted = false;
    CHECK(AutoMountLittleFs(dev, false) == LittleFsAutoMount::kDisabled);
    dev.arduinoLibrary = true;
    CHECK(AutoMountLittleFs(dev, true) == LittleFsAutoMount::kArduinoLibrary);
    CHECK(dev.formats == 0);
    CHECK(dev.mounts == 0);
  }
}

TEST_CASE("LittleFS auto-mount without a data partition", "[prosdk][platform][storage]") {
  FakeDevice dev;
  dev.partition = false;
  CHECK(AutoMountLittleFs(dev, true) == LittleFsAutoMount::kNoPartition);
  CHECK(dev.blankChecks == 0);
  CHECK(dev.formats == 0);
  CHECK(dev.mounts == 0);
}

TEST_CASE("LittleFS auto-mount formats a blank partition, then mounts it without formatting",
          "[prosdk][platform][storage]") {
  FakeDevice dev;
  CHECK(AutoMountLittleFs(dev, true) == LittleFsAutoMount::kFormatted);
  CHECK(dev.formats == 1);
  CHECK(dev.mounts == 1);
  CHECK(dev.mounted);

  // the next boot: the file system is there, mounted as it is
  dev.mounted = false;
  CHECK(AutoMountLittleFs(dev, true) == LittleFsAutoMount::kMounted);
  CHECK(dev.formats == 1);
  CHECK(dev.mounts == 2);
  CHECK(dev.mounted);
}

TEST_CASE("LittleFS auto-mount never formats a partition that holds data",
          "[prosdk][platform][storage]") {
  FakeDevice dev;
  dev.content = Content::kOther;  // SPIFFS, another program's data, a damaged file system
  CHECK(AutoMountLittleFs(dev, true) == LittleFsAutoMount::kNotBlank);
  CHECK(dev.formats == 0);
  CHECK(dev.mounts == 1);
  CHECK_FALSE(dev.mounted);
  CHECK(dev.content == Content::kOther);
}

TEST_CASE("LittleFS auto-mount errors format nothing more", "[prosdk][platform][storage]") {
  SECTION("mount error on a used partition") {
    FakeDevice dev;
    dev.content = Content::kLittleFs;
    dev.mountError = true;
    CHECK(AutoMountLittleFs(dev, true) == LittleFsAutoMount::kFailed);
    CHECK(dev.formats == 0);
  }
  SECTION("format error on a blank partition") {
    FakeDevice dev;
    dev.formatFails = true;
    CHECK(AutoMountLittleFs(dev, true) == LittleFsAutoMount::kFailed);
    CHECK(dev.formats == 1);
    CHECK(dev.mounts == 0);
  }
  SECTION("mount error after formatting a blank partition") {
    FakeDevice dev;
    dev.mountError = true;
    CHECK(AutoMountLittleFs(dev, true) == LittleFsAutoMount::kFailed);
    CHECK(dev.formats == 1);
    CHECK_FALSE(dev.mounted);
  }
}

TEST_CASE("Log auto-mount switch; the host never mounts", "[prosdk][platform][storage]") {
  CHECK(prosdk::platform::GetLogAutoMount());
  prosdk::platform::SetLogAutoMount(false);
  CHECK_FALSE(prosdk::platform::GetLogAutoMount());
  // the host log directory does not depend on it (no partition)
  std::string dir = prosdk::platform::GetLogDirectory();
  CHECK_FALSE(dir.empty());
  prosdk::platform::SetLogAutoMount(true);
  CHECK(prosdk::platform::GetLogAutoMount());
  CHECK(prosdk::platform::GetLogDirectory() == dir);
  CHECK(prosdk::platform::detail::GetLittleFsAutoMountResult() == LittleFsAutoMount::kNotTried);
  CHECK_FALSE(prosdk::platform::IsLittleFsMounted());
}
