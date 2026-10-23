// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: SPI buses on the ESP-IDF spi_master driver (prosdk/platform/hal/SPI.hpp,
// design/boardoza/F1-spi.md). Bus 0 is SPI2 (FSPI) and bus 1 is SPI3; SPI0/SPI1 (flash and PSRAM)
// are never touched. A bus starts on its first user with the pins from
// prosdk::platform::SetSPIPins (bus 0 defaults to SCK 17 / MOSI 15 / MISO 16, the Boardoza
// Pulse J5; bus 1 has no default), DMA (SPI_DMA_CH_AUTO) and 4092-byte transfers, and
// stops when its last user leaves (no Initialize hook: lazy, like the serial ports). SCK, MOSI,
// MISO and every chip select are claimed in the HAL pin table (ClaimPin) while in use, as the
// serial ports do, and return to gpio_reset_pin state afterwards. Chip selects are the
// peripheral's hardware CS signals: 6 on SPI2, 3 on SPI3 (SOC_SPI_PERIPH_CS_NUM). A device without
// chip select must be alone on its bus. Transfers of at most 64 bytes (the CPU FIFO,
// SOC_SPI_MAXIMUM_BUFFER_SIZE) are polled, longer ones use the interrupt and DMA; a segment over
// 4092 bytes is split, and the chip select stays active between pieces and segments
// (SPI_TRANS_CS_KEEP_ACTIVE with the bus acquired). SPI is not gated by the output gate or the
// emergency stop, like I2C and serial.

#include "prosdk/platform/hal/SPI.hpp"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <soc/soc_caps.h>

#include <algorithm>
#include <format>
#include <mutex>
#include <tuple>

#include "Esp32Internal.hpp"
#include "SpiInternal.hpp"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/platform/HalConfig.hpp"

namespace {

namespace esp = ::prosdk::hal::esp32;
using ::prosdk::hal::HAL_HandleEnum;
using ::prosdk::hal::MakeError;
using ::prosdk::platform::hal::kInvalidSpiHandle;
using ::prosdk::platform::hal::SpiDeviceConfig;
using ::prosdk::platform::hal::detail::SpiHostOf;

constexpr int kNumBuses = prosdk::platform::kNumSPIBuses;
constexpr int kMaxDevices = SOC_SPI_MAX_CS_NUM;  // device slots per bus; handle = bus * 6 + slot
constexpr int kMaxTransfer = 4092;  // bytes per transaction (SPI_MAX_DMA_LEN, spi_common.h)
constexpr int kPollingMax = SOC_SPI_MAXIMUM_BUFFER_SIZE;  // 64 bytes: polled, no interrupt
// WPILib 2027 has no SPI handle type; the pin table only needs a type that frees the claim
constexpr HAL_HandleEnum kClaimType = HAL_HandleEnum::VENDOR;

struct Device {
  bool open = false;
  int cs = -1;
  SpiDeviceConfig config;
  spi_device_handle_t dev = nullptr;
  HAL_DigitalHandle csClaim = HAL_INVALID_HANDLE;
};

struct Bus {
  std::mutex mutex;  // everything below, and the transfers of the bus's devices
  int users = 0;     // open devices + RetainSpiBus users; the bus runs while > 0
  bool noCs = false;  // carries the device without chip select
  bool misoUsed = false;
  int pins[3] = {-1, -1, -1};  // SCK, MOSI, MISO
  HAL_DigitalHandle claims[3] = {HAL_INVALID_HANDLE, HAL_INVALID_HANDLE, HAL_INVALID_HANDLE};
  Device devices[kMaxDevices];
};

Bus gBuses[kNumBuses];

const char* HostName(int bus) {
  return bus == 0 ? "SPI2" : "SPI3";
}

int MaxChipSelects(int bus) {
  return SOC_SPI_PERIPH_CS_NUM(static_cast<int>(SpiHostOf(bus)));
}

// kMaxTransfer zero bytes in DMA-capable memory, sent by segments without tx (allocated once,
// on the first such segment)
const uint8_t* Zeros() {
  static const uint8_t* zeros =
      static_cast<const uint8_t*>(heap_caps_calloc(kMaxTransfer, 1, MALLOC_CAP_DMA));
  return zeros;
}

// bus.mutex held
void ReleaseBusPins(Bus& bus) {
  for (int i = 0; i < 3; ++i) {
    if (bus.claims[i] != HAL_INVALID_HANDLE) {
      gpio_reset_pin(static_cast<gpio_num_t>(bus.pins[i]));
      esp::ReleasePin(bus.claims[i], kClaimType);
    }
    bus.claims[i] = HAL_INVALID_HANDLE;
    bus.pins[i] = -1;
  }
}

// bus.mutex held. Starts the bus when it has no users yet.
int32_t StartBus(int index, Bus& bus) {
  if (bus.users > 0) {
    return HAL_SUCCESS;
  }
  prosdk::platform::SPIConfig config = prosdk::platform::GetSPIConfig(index);
  if (config.sck < 0 || config.mosi < 0) {
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     std::format("SPI bus {} has no pins: call "
                                 "prosdk::platform::SetSPIPins({}, sck, mosi, miso) first",
                                 index, index));
  }
  for (auto [i, gpio, role] : {std::tuple{0, config.sck, "SCK"},
                               std::tuple{1, config.mosi, "MOSI"},
                               std::tuple{2, config.miso, "MISO"}}) {
    if (gpio < 0) {
      continue;
    }
    auto claimed = esp::ClaimPin(gpio, kClaimType, "SPI",
                                 std::format("SPI bus {} ({} {})", index, HostName(index), role));
    if (!claimed) {
      ReleaseBusPins(bus);
      return claimed.error();
    }
    bus.claims[i] = *claimed;
    bus.pins[i] = gpio;
  }
  spi_bus_config_t bc{};
  bc.mosi_io_num = config.mosi;
  bc.miso_io_num = config.miso;
  bc.sclk_io_num = config.sck;
  bc.quadwp_io_num = -1;
  bc.quadhd_io_num = -1;
  bc.data4_io_num = -1;
  bc.data5_io_num = -1;
  bc.data6_io_num = -1;
  bc.data7_io_num = -1;
  bc.max_transfer_sz = kMaxTransfer;
  bc.flags = SPICOMMON_BUSFLAG_MASTER;
  esp_err_t err = spi_bus_initialize(SpiHostOf(index), &bc, SPI_DMA_CH_AUTO);
  if (err != ESP_OK) {
    ReleaseBusPins(bus);
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     std::format("SPI bus {} ({}) could not start ({}; is Arduino's SPI library "
                                 "using it?)",
                                 index, HostName(index), esp::EspErrName(err)));
  }
  bus.misoUsed = config.miso >= 0;
  return HAL_SUCCESS;
}

// bus.mutex held. Stops the bus when it has no users left.
void StopBusIfUnused(int index, Bus& bus) {
  if (bus.users > 0) {
    return;
  }
  esp_err_t err = spi_bus_free(SpiHostOf(index));
  if (err != ESP_OK) {
    // a RetainSpiBus user left its own device on the bus
    esp::Log(std::format("SPI bus {} ({}) could not stop ({})", index, HostName(index),
                         esp::EspErrName(err)));
  }
  ReleaseBusPins(bus);
  bus.noCs = false;
  bus.misoUsed = false;
}

// bus.mutex held
esp_err_t AddDevice(int index, const Bus& bus, Device& d) {
  spi_device_interface_config_t dc{};
  dc.mode = static_cast<uint8_t>(d.config.mode);
  dc.clock_source = SPI_CLK_SRC_DEFAULT;
  dc.clock_speed_hz = d.config.clockHz;
  dc.spics_io_num = d.cs;
  dc.queue_size = 1;
  dc.flags = d.config.csActiveHigh ? SPI_DEVICE_POSITIVE_CS : 0;
  if (!bus.misoUsed) {
    // nothing is read on a transmit-only bus: skip the full-duplex read-timing check
    dc.flags |= SPI_DEVICE_NO_DUMMY;
  }
  return spi_bus_add_device(SpiHostOf(index), &dc, &d.dev);
}

bool ValidConfig(const SpiDeviceConfig& config) {
  return config.mode >= 0 && config.mode <= 3 && config.clockHz > 0;
}

// The device of a handle with its bus locked; nullptr (no lock) for an invalid or closed handle.
Device* Lock(int32_t handle, std::unique_lock<std::mutex>* lock, int* index) {
  if (handle < 0 || handle >= kNumBuses * kMaxDevices) {
    return nullptr;
  }
  *index = handle / kMaxDevices;
  Bus& bus = gBuses[*index];
  std::unique_lock<std::mutex> l{bus.mutex};
  Device& d = bus.devices[handle % kMaxDevices];
  if (!d.open) {
    return nullptr;
  }
  *lock = std::move(l);
  return &d;
}

}  // namespace

namespace prosdk::platform::hal {

namespace detail {

spi_host_device_t SpiHostOf(int bus) {
  switch (bus) {
    case 0:
      return SPI2_HOST;
    case 1:
      return SPI3_HOST;
    default:
      return SPI_HOST_MAX;
  }
}

bool RetainSpiBus(int bus, int32_t* status) {
  ::prosdk::hal::init::CheckInit();
  if (bus < 0 || bus >= kNumBuses) {
    *status = ::prosdk::hal::MakeErrorIndexOutOfRange(HAL_RESOURCE_OUT_OF_RANGE,
                                                      "Invalid Index for SPI", 0, kNumBuses - 1,
                                                      bus);
    return false;
  }
  Bus& b = gBuses[bus];
  std::scoped_lock lock{b.mutex};
  if (b.noCs) {
    *status = MakeError(HAL_RESOURCE_IS_ALLOCATED,
                        std::format("SPI bus {} carries a device without chip select, which must "
                                    "be alone on its bus",
                                    bus));
    return false;
  }
  *status = StartBus(bus, b);
  if (*status != HAL_SUCCESS) {
    return false;
  }
  ++b.users;
  return true;
}

void ReleaseSpiBus(int bus) {
  if (bus < 0 || bus >= kNumBuses) {
    return;
  }
  Bus& b = gBuses[bus];
  std::scoped_lock lock{b.mutex};
  if (b.users > 0) {
    --b.users;
  }
  StopBusIfUnused(bus, b);
}

}  // namespace detail

int32_t OpenSpiDevice(int bus, int cs, const SpiDeviceConfig& config, int32_t* status) {
  ::prosdk::hal::init::CheckInit();
  if (bus < 0 || bus >= kNumBuses) {
    *status = ::prosdk::hal::MakeErrorIndexOutOfRange(HAL_RESOURCE_OUT_OF_RANGE,
                                                      "Invalid Index for SPI", 0, kNumBuses - 1,
                                                      bus);
    return kInvalidSpiHandle;
  }
  if (cs < -1 || !ValidConfig(config)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE,
                        std::format("SPI bus {}: invalid chip select {} or configuration", bus,
                                    cs));
    return kInvalidSpiHandle;
  }
  Bus& b = gBuses[bus];
  std::scoped_lock lock{b.mutex};
  if (b.noCs || (cs < 0 && b.users > 0)) {
    *status = MakeError(HAL_RESOURCE_IS_ALLOCATED,
                        std::format("SPI bus {}: a device without chip select must be alone on "
                                    "its bus",
                                    bus));
    return kInvalidSpiHandle;
  }
  if (cs >= 0 && b.users >= MaxChipSelects(bus)) {
    *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                        std::format("SPI bus {} ({}) has no free chip select (at most {})", bus,
                                    HostName(bus), MaxChipSelects(bus)));
    return kInvalidSpiHandle;
  }
  auto free = std::find_if(std::begin(b.devices), std::end(b.devices),
                           [](const Device& d) { return !d.open; });
  if (free == std::end(b.devices)) {
    *status = MakeError(HAL_NO_AVAILABLE_RESOURCES, std::format("SPI bus {} is full", bus));
    return kInvalidSpiHandle;
  }
  *status = StartBus(bus, b);
  if (*status != HAL_SUCCESS) {
    return kInvalidSpiHandle;
  }
  ++b.users;
  Device& d = *free;
  if (cs >= 0) {
    auto claimed = esp::ClaimPin(cs, kClaimType, "SPI",
                                 std::format("SPI bus {} ({} CS)", bus, HostName(bus)));
    if (!claimed) {
      *status = claimed.error();
      --b.users;
      StopBusIfUnused(bus, b);
      return kInvalidSpiHandle;
    }
    d.csClaim = *claimed;
  }
  d.cs = cs;
  d.config = config;
  esp_err_t err = AddDevice(bus, b, d);
  if (err != ESP_OK) {
    esp::ReleasePin(d.csClaim, kClaimType);
    d = Device{};
    --b.users;
    StopBusIfUnused(bus, b);
    *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                        std::format("SPI bus {} CS {}: the device could not be added ({}; mode {}, "
                                    "{} Hz)",
                                    bus, cs, esp::EspErrName(err), config.mode, config.clockHz));
    return kInvalidSpiHandle;
  }
  d.open = true;
  if (cs < 0) {
    b.noCs = true;
  }
  *status = HAL_SUCCESS;
  return bus * kMaxDevices + static_cast<int32_t>(free - std::begin(b.devices));
}

void ConfigureSpiDevice(int32_t handle, const SpiDeviceConfig& config, int32_t* status) {
  std::unique_lock<std::mutex> lock;
  int bus = 0;
  Device* d = Lock(handle, &lock, &bus);
  if (d == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  if (!ValidConfig(config)) {
    *status = HAL_PARAMETER_OUT_OF_RANGE;
    return;
  }
  const SpiDeviceConfig previous = d->config;
  if (d->dev != nullptr && config.mode == previous.mode && config.clockHz == previous.clockHz &&
      config.csActiveHigh == previous.csActiveHigh) {
    *status = HAL_SUCCESS;
    return;
  }
  // the device's clock, mode and CS polarity are fixed when it is added: remove and add it again
  if (d->dev != nullptr) {
    spi_bus_remove_device(d->dev);
    d->dev = nullptr;
  }
  d->config = config;
  esp_err_t err = AddDevice(bus, gBuses[bus], *d);
  if (err == ESP_OK) {
    *status = HAL_SUCCESS;
    return;
  }
  d->config = previous;
  esp_err_t back = AddDevice(bus, gBuses[bus], *d);
  *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE,
                      std::format("SPI bus {} CS {}: mode {}, {} Hz refused ({}){}", bus, d->cs,
                                  config.mode, config.clockHz, esp::EspErrName(err),
                                  back == ESP_OK ? "" : "; the device is unusable"));
  if (back != ESP_OK) {
    d->dev = nullptr;
  }
}

bool SpiTransfer(int32_t handle, std::span<const prosdk::SpiSegment> segments) {
  std::unique_lock<std::mutex> lock;
  int bus = 0;
  Device* d = Lock(handle, &lock, &bus);
  if (d == nullptr || d->dev == nullptr) {
    return false;
  }
  int pieces = 0;
  bool needZeros = false;
  for (const prosdk::SpiSegment& s : segments) {
    if (s.size < 0) {
      return false;
    }
    pieces += (s.size + kMaxTransfer - 1) / kMaxTransfer;
    needZeros = needZeros || (s.tx == nullptr && s.size > 0);
  }
  if (pieces == 0) {
    return true;
  }
  const uint8_t* zeros = needZeros ? Zeros() : nullptr;
  if (needZeros && zeros == nullptr) {
    esp::Log("SPI: no memory for the read buffer");
    return false;
  }
  // several pieces: keep other devices off the bus and the chip select active in between
  const bool hold = pieces > 1;
  if (hold && spi_device_acquire_bus(d->dev, portMAX_DELAY) != ESP_OK) {
    return false;
  }
  bool ok = true;
  int piece = 0;
  for (const prosdk::SpiSegment& s : segments) {
    for (int offset = 0; ok && offset < s.size; offset += kMaxTransfer) {
      const int n = std::min(kMaxTransfer, s.size - offset);
      ++piece;
      spi_transaction_t t{};
      t.length = static_cast<size_t>(n) * 8;
      t.tx_buffer = s.tx != nullptr ? s.tx + offset : zeros;
      t.rx_buffer = s.rx != nullptr ? s.rx + offset : nullptr;
      if (hold && d->cs >= 0 && piece < pieces) {
        t.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
      }
      esp_err_t err = n <= kPollingMax ? spi_device_polling_transmit(d->dev, &t)
                                       : spi_device_transmit(d->dev, &t);
      ok = err == ESP_OK;
    }
  }
  if (hold) {
    spi_device_release_bus(d->dev);
  }
  return ok;
}

void CloseSpiDevice(int32_t handle) {
  std::unique_lock<std::mutex> lock;
  int bus = 0;
  Device* d = Lock(handle, &lock, &bus);
  if (d == nullptr) {
    return;
  }
  Bus& b = gBuses[bus];
  if (d->dev != nullptr) {
    spi_bus_remove_device(d->dev);
  }
  if (d->cs >= 0) {
    gpio_reset_pin(static_cast<gpio_num_t>(d->cs));
    esp::ReleasePin(d->csClaim, kClaimType);
  } else {
    b.noCs = false;
  }
  *d = Device{};
  --b.users;
  StopBusIfUnused(bus, b);
}

}  // namespace prosdk::platform::hal
