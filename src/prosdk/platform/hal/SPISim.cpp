// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Host simulation of the SPI backend (prosdk/platform/hal/SPI.hpp) and its test hooks
// (prosdk/simulation/SpiSim.hpp). It applies the same rules as the ESP32-S3 backend (bus range,
// one owner per chip select, a device without chip select alone on its bus, at most 6 chip
// selects on bus 0 and 3 on bus 1) but needs no pins. Compiled into both builds by the
// platform/hal glob; empty on the robot.

#include "prosdk/platform/config.hpp"

#if !PROSDK_TARGET_HW

#include <algorithm>
#include <format>
#include <map>
#include <mutex>
#include <vector>

#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/platform/HalConfig.hpp"
#include "prosdk/platform/hal/SPI.hpp"
#include "prosdk/simulation/SpiSim.hpp"

namespace {

using prosdk::platform::hal::SpiDeviceConfig;

constexpr int kMaxDevices = 6;                // handle = bus * kMaxDevices + slot
constexpr int kMaxChipSelects[] = {6, 3};     // as SOC_SPI_PERIPH_CS_NUM (SPI2, SPI3)

struct Device {
  bool open = false;
  int cs = -1;
  SpiDeviceConfig config;
};

struct Callback {
  int bus;
  int cs;
  prosdk::sim::SpiTransferFn fn;
};

// recursive: CancelSpiDevice from a destructor running inside a callback does not deadlock
std::recursive_mutex gMutex;
Device gDevices[prosdk::platform::kNumSPIBuses][kMaxDevices];
std::map<int, Callback> gCallbacks;
int gNextId = 1;

bool ValidConfig(const SpiDeviceConfig& config) {
  return config.mode >= 0 && config.mode <= 3 && config.clockHz > 0;
}

Device* Find(int32_t handle) {
  if (handle < 0 || handle >= prosdk::platform::kNumSPIBuses * kMaxDevices) {
    return nullptr;
  }
  Device& d = gDevices[handle / kMaxDevices][handle % kMaxDevices];
  return d.open ? &d : nullptr;
}

}  // namespace

namespace prosdk::platform::hal {

int32_t OpenSpiDevice(int bus, int cs, const SpiDeviceConfig& config, int32_t* status) {
  using ::prosdk::hal::MakeError;
  if (bus < 0 || bus >= kNumSPIBuses) {
    *status = ::prosdk::hal::MakeErrorIndexOutOfRange(
        HAL_RESOURCE_OUT_OF_RANGE, "Invalid Index for SPI", 0, kNumSPIBuses - 1, bus);
    return kInvalidSpiHandle;
  }
  if (cs < -1 || !ValidConfig(config)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE,
                        std::format("SPI bus {}: invalid chip select {} or configuration", bus,
                                    cs));
    return kInvalidSpiHandle;
  }
  std::scoped_lock lock{gMutex};
  int users = 0;
  bool noCs = false;
  for (const Device& d : gDevices[bus]) {
    if (d.open) {
      ++users;
      noCs = noCs || d.cs < 0;
    }
  }
  if (cs >= 0) {
    for (const auto& busDevices : gDevices) {
      for (const Device& d : busDevices) {
        if (d.open && d.cs == cs) {
          *status = MakeError(HAL_RESOURCE_IS_ALLOCATED,
                              std::format("SPI bus {}: chip select GPIO {} is already open", bus,
                                          cs));
          return kInvalidSpiHandle;
        }
      }
    }
  }
  if (noCs || (cs < 0 && users > 0)) {
    *status = MakeError(HAL_RESOURCE_IS_ALLOCATED,
                        std::format("SPI bus {}: a device without chip select must be alone on "
                                    "its bus",
                                    bus));
    return kInvalidSpiHandle;
  }
  if (cs >= 0 && users >= kMaxChipSelects[bus]) {
    *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                        std::format("SPI bus {} has no free chip select (at most {})", bus,
                                    kMaxChipSelects[bus]));
    return kInvalidSpiHandle;
  }
  for (int slot = 0; slot < kMaxDevices; ++slot) {
    Device& d = gDevices[bus][slot];
    if (!d.open) {
      d.open = true;
      d.cs = cs;
      d.config = config;
      *status = HAL_SUCCESS;
      return bus * kMaxDevices + slot;
    }
  }
  *status = MakeError(HAL_NO_AVAILABLE_RESOURCES, std::format("SPI bus {} is full", bus));
  return kInvalidSpiHandle;
}

void ConfigureSpiDevice(int32_t handle, const SpiDeviceConfig& config, int32_t* status) {
  std::scoped_lock lock{gMutex};
  Device* d = Find(handle);
  if (d == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  if (!ValidConfig(config)) {
    *status = HAL_PARAMETER_OUT_OF_RANGE;
    return;
  }
  d->config = config;
  *status = HAL_SUCCESS;
}

bool SpiTransfer(int32_t handle, std::span<const prosdk::SpiSegment> segments) {
  std::scoped_lock lock{gMutex};
  Device* d = Find(handle);
  if (d == nullptr) {
    return false;
  }
  const int bus = handle / kMaxDevices;
  std::vector<uint8_t> tx;
  for (const prosdk::SpiSegment& s : segments) {
    if (s.size < 0) {
      return false;
    }
    if (s.tx != nullptr) {
      tx.insert(tx.end(), s.tx, s.tx + s.size);
    } else {
      tx.insert(tx.end(), static_cast<size_t>(s.size), uint8_t{0});
    }
  }
  if (tx.empty()) {
    return true;  // nothing to clock: no chip-select assertion (as on the ESP32)
  }
  std::vector<uint8_t> rx(tx.size(), 0);
  // copy: a callback may cancel itself
  std::vector<prosdk::sim::SpiTransferFn> fns;
  for (const auto& [id, cb] : gCallbacks) {
    if (cb.bus == bus && cb.cs == d->cs) {
      fns.push_back(cb.fn);
    }
  }
  for (const auto& fn : fns) {
    fn(std::span<const uint8_t>{tx}, std::span<uint8_t>{rx});
  }
  size_t offset = 0;
  for (const prosdk::SpiSegment& s : segments) {
    if (s.rx != nullptr) {
      std::copy_n(rx.begin() + offset, s.size, s.rx);
    }
    offset += static_cast<size_t>(s.size);
  }
  return true;
}

void CloseSpiDevice(int32_t handle) {
  std::scoped_lock lock{gMutex};
  Device* d = Find(handle);
  if (d != nullptr) {
    *d = Device{};
  }
}

}  // namespace prosdk::platform::hal

namespace prosdk::sim {

int RegisterSpiDevice(int bus, int cs, SpiTransferFn fn) {
  std::scoped_lock lock{gMutex};
  int id = gNextId++;
  gCallbacks.emplace(id, Callback{bus, cs, std::move(fn)});
  return id;
}

void CancelSpiDevice(int id) {
  std::scoped_lock lock{gMutex};
  gCallbacks.erase(id);
}

bool GetSpiDeviceConfig(int bus, int cs, prosdk::platform::hal::SpiDeviceConfig* config) {
  if (bus < 0 || bus >= prosdk::platform::kNumSPIBuses) {
    return false;
  }
  std::scoped_lock lock{gMutex};
  for (const Device& d : gDevices[bus]) {
    if (d.open && d.cs == cs) {
      *config = d.config;
      return true;
    }
  }
  return false;
}

}  // namespace prosdk::sim

#endif  // !PROSDK_TARGET_HW
