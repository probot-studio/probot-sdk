// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: GPIO ownership, the output gate and the emergency stop
// (Esp32Internal.hpp).

#include <driver/gpio.h>
#include <esp32-hal-periman.h>
#include <esp_err.h>
#include <esp_private/esp_gpio_reserve.h>
#include <esp_rom_gpio.h>
#include <esp_timer.h>
#include <soc/gpio_sig_map.h>

#include <chrono>
#include <cstdio>
#include <format>
#include <string>
#include <thread>

#include "Esp32Internal.hpp"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/monotonic_clock.hpp"
#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/RobotHost.hpp"

namespace prosdk::hal::esp32 {

DigitalHandleResource<HAL_DigitalHandle, SmartIo, kNumSmartIo>* smartIoHandles;

namespace can {
// Weak reference (design/size.md): CAN.cpp, with the TWAI driver, is linked only when the sketch
// uses CAN; without it there is no CAN transmission to stop.
[[gnu::weak]] void OnEmergencyStop();
}  // namespace can

namespace sixstep {
// Weak references, as for CAN: SixStep.cpp is linked only when the sketch opens a six-step motor.
[[gnu::weak]] bool ApplyGate(bool enabled, bool blocking);
[[gnu::weak]] void OnEmergencyStop();
}  // namespace sixstep

namespace {

constexpr uint64_t Bit(int gpio) {
  return uint64_t{1} << gpio;
}

// USB D-/D+ (USB-Serial-JTAG: flashing and the USB console) and UART0 TX/RX (Serial, the boot
// console that probot-core prints to).
constexpr uint64_t kUsbPins = Bit(19) | Bit(20);
constexpr uint64_t kConsolePins = Bit(43) | Bit(44);

constinit std::atomic<uint64_t> gOutputPins{0};
constinit std::atomic<bool> gEStop{false};
esp_timer_handle_t gGateTimer = nullptr;

// Emergency stop: core deleted its user task and runs this on a fresh core-1 task. Only register
// writes and atomics: route every HAL output pin to the plain GPIO output and drive it low, so
// MCPWM, LEDC and RMT signals stop whatever their drivers still do.
void OnEmergencyStop() {
  gEStop.store(true, std::memory_order_release);
  if (sixstep::OnEmergencyStop != nullptr) {
    sixstep::OnEmergencyStop();  // first: its Hall interrupt could re-drive pins after the loop
  }
  uint64_t pins = gOutputPins.load(std::memory_order_acquire);
  for (int gpio = 0; gpio < kNumSmartIo; ++gpio) {
    if ((pins & Bit(gpio)) != 0) {
      // output register low first, then route: no glitch from a stale GPIO_OUT bit
      gpio_set_level(static_cast<gpio_num_t>(gpio), 0);
      esp_rom_gpio_connect_out_signal(gpio, SIG_GPIO_OUT_IDX, false, false);
    }
  }
  if (can::OnEmergencyStop != nullptr) {
    can::OnEmergencyStop();
  }
}

// Never blocks the esp_timer task: if the PWM table is busy the next tick retries.
void GateTimerCallback(void*) {
  if (!EStopped()) {
    bool enabled = OutputsEnabled();
    pwm::ApplyGate(enabled, false);
    dio::ApplyGate(enabled, false);
    if (sixstep::ApplyGate != nullptr) {
      sixstep::ApplyGate(enabled, false);
    }
  }
}

void GatePreCycle(void*) {
  UpdateOutputGate();
}

}  // namespace

bool IsValidGpio(int32_t gpio) {
  return gpio >= 0 && gpio < kNumSmartIo && !(gpio >= 22 && gpio <= 25);
}

HAL_Status CheckPinUsable(int32_t gpio, std::string_view what) {
  if (gpio < 0 || gpio >= kNumSmartIo) {
    return MakeErrorIndexOutOfRange(HAL_RESOURCE_OUT_OF_RANGE,
                                    std::format("Invalid Index for {}", what), 0,
                                    kNumSmartIo - 1, gpio);
  }
  if (!IsValidGpio(gpio)) {
    return MakeError(HAL_RESOURCE_OUT_OF_RANGE,
                     std::format("{}: GPIO {} does not exist on the ESP32-S3 (GPIO 0-21, 26-48)",
                                 what, gpio));
  }
  uint64_t pinMask = Bit(gpio);
  if ((pinMask & kUsbPins) != 0) {
    return MakeError(HAL_RESOURCE_IS_ALLOCATED,
                     std::format("{}: GPIO {} is reserved for USB (D-/D+: flashing and the USB "
                                 "console)",
                                 what, gpio));
  }
  if ((pinMask & kConsolePins) != 0) {
    return MakeError(HAL_RESOURCE_IS_ALLOCATED,
                     std::format("{}: GPIO {} is reserved for UART0 (Serial, the console)", what,
                                 gpio));
  }
  if (esp_gpio_is_reserved(pinMask)) {
    return MakeError(HAL_RESOURCE_IS_ALLOCATED,
                     std::format("{}: GPIO {} is reserved by this module's flash or PSRAM", what,
                                 gpio));
  }
  if ((pinMask & platform::GetCoreReservedPins()) != 0) {
    return MakeErrorPreviouslyAllocated(
        HAL_RESOURCE_IS_ALLOCATED, what, gpio,
        "probot-core (NEOPIXEL_PIN, PROBOT_RSL_PIN, PROBOT_ESTOP_ENABLE_PIN or a battery sensor "
        "pin)");
  }
  peripheral_bus_type_t bus = perimanGetPinBusType(static_cast<uint8_t>(gpio));
  if (bus != ESP32_BUS_TYPE_INIT) {
    return MakeError(HAL_RESOURCE_IS_ALLOCATED,
                     std::format("{}: GPIO {} is in use by another Arduino driver ({})", what,
                                 gpio, perimanGetTypeName(bus)));
  }
  return HAL_SUCCESS;
}

prosdk::cxx::expected<std::pair<HAL_DigitalHandle, std::shared_ptr<SmartIo>>, HAL_Status>
AllocateSmartIo(int32_t gpio, HAL_HandleEnum type, std::string_view what) {
  if (gpio < 0 || gpio >= kNumSmartIo) {
    return prosdk::cxx::unexpected(CheckPinUsable(gpio, what));
  }
  auto resource = smartIoHandles->Allocate(static_cast<int16_t>(gpio), type, what);
  if (!resource) {
    return resource;
  }
  HAL_Status status = CheckPinUsable(gpio, what);
  if (status != HAL_SUCCESS) {
    smartIoHandles->Free(resource->first, type);
    return prosdk::cxx::unexpected(status);
  }
  resource->second->channel = static_cast<uint8_t>(gpio);
  return resource;
}

void FreeSmartIo(HAL_DigitalHandle handle, HAL_HandleEnum type,
                 std::shared_ptr<SmartIo>&& port, const char* what) {
  smartIoHandles->Free(handle, type);
  // Wait for no other object to hold this handle (systemcore does the same).
  auto start = monotonic_clock::now();
  while (port.use_count() != 1) {
    if (start + std::chrono::seconds(1) < monotonic_clock::now()) {
      Log(std::format("{} handle free timeout", what), platform::ConsoleLevel::kWarning);
      break;
    }
    std::this_thread::yield();
  }
  port.reset();
}

prosdk::cxx::expected<HAL_DigitalHandle, HAL_Status> ClaimPin(int32_t gpio, HAL_HandleEnum type,
                                                            std::string_view what,
                                                            std::string_view owner) {
  auto resource = AllocateSmartIo(gpio, type, what);
  if (!resource) {
    return prosdk::cxx::unexpected(resource.error());
  }
  auto& [handle, port] = *resource;
  port->mode = PinMode::kPeripheral;
  port->previousAllocation = owner;
  return handle;
}

void ReleasePin(HAL_DigitalHandle handle, HAL_HandleEnum type) {
  if (handle != HAL_INVALID_HANDLE) {
    smartIoHandles->Free(handle, type);
  }
}

void ReleaseArduinoPin(int32_t gpio) {
  if (IsValidGpio(gpio) &&
      perimanGetPinBusType(static_cast<uint8_t>(gpio)) != ESP32_BUS_TYPE_INIT) {
    perimanSetPinBus(static_cast<uint8_t>(gpio), ESP32_BUS_TYPE_INIT, nullptr, -1, -1);
  }
}

void SetOutputPin(int32_t gpio, bool isOutput) {
  if (!IsValidGpio(gpio)) {
    return;
  }
  if (isOutput) {
    gOutputPins.fetch_or(Bit(gpio), std::memory_order_acq_rel);
  } else {
    gOutputPins.fetch_and(~Bit(gpio), std::memory_order_acq_rel);
  }
}

bool EStopped() {
  return gEStop.load(std::memory_order_acquire) || platform::EmergencyStopped();
}

// Without prosdk's framework hooks (another framework owns core's hooks) the e-stop callback
// and the pre-cycle gate never run, so the gate stays closed. A halted robot program (constructor
// or loop exception, prosdk/platform/RobotHost.hpp) keeps it closed too.
// core's outputsEnabled() does not check the Driver Station link, so isAttached() is required
// as well: it is what the control word's enabled/DS-attached bits come from (DriverStation.cpp
// MakeControlWord), so the gate closes when the framework sees the robot disabled after the DS
// owner slot is released (PROBOT_DS_OWNER_TIMEOUT_MS, 5 s), not only at core's DS timeout stop.
bool OutputsEnabled() {
  const platform::CoreOps& ops = platform::GetCoreOps();
  return !EStopped() && platform::FrameworkInstalled() && !platform::RobotProgramHalted() &&
         ops.outputsEnabled() && ops.isAttached();
}

void UpdateOutputGate() {
  bool enabled = OutputsEnabled();
  pwm::ApplyGate(enabled, true);
  dio::ApplyGate(enabled, true);
  if (sixstep::ApplyGate != nullptr) {
    sixstep::ApplyGate(enabled, true);
  }
}

void Log(std::string_view text, platform::ConsoleLevel level) {
  std::string line = "[prosdk hal] ";
  line.append(text);
  line.push_back('\n');
  platform::ConsoleWrite(line, level);
}

const char* EspErrName(int err) {
  return esp_err_to_name(static_cast<esp_err_t>(err));
}

}  // namespace prosdk::hal::esp32

namespace prosdk::hal::init {
void InitializeSmartIo() {
  using namespace prosdk::hal::esp32;
  static DigitalHandleResource<HAL_DigitalHandle, SmartIo, kNumSmartIo> handles;
  smartIoHandles = &handles;

  platform::AddEmergencyStopCallback(&OnEmergencyStop);
  platform::AddCycleCallback(platform::CyclePhase::kPre, &GatePreCycle, nullptr);

  esp_timer_create_args_t args{};
  args.callback = &GateTimerCallback;
  args.name = "prosdk-gate";
  if (esp_timer_create(&args, &gGateTimer) == ESP_OK) {
    esp_timer_start_periodic(gGateTimer, 10000);  // 10 ms
  } else {
    Log("output gate timer could not be created; the gate follows core's loop only",
        platform::ConsoleLevel::kWarning);
  }
}
}  // namespace prosdk::hal::init
