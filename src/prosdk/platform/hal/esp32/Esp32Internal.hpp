// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Shared internals of the ESP32-S3 HAL backend (design/hal-esp32.md).
//
// Channel model: channel number = GPIO number for every pin API (DIO, PWM, Counter, Encoder,
// DutyCycle, AnalogInput, AddressableLED, pulse I/O). One DigitalHandleResource indexed by GPIO
// (smartIoHandles) gives each pin exactly one owner, like systemcore's SmartIo table. A pin is
// checked against the chip (GPIO 22-25 do not exist), the module (flash/PSRAM pins), the board
// (USB D-/D+, UART0 console) and probot-core (pins named by its configuration macros) before it is
// handed out.
//
// Output safety (PORT.md section 8): servo/ESC PWM pulses and digital PWM duty cycles (LEDC)
// only leave the chip while probot-core reports outputsEnabled() (started RUN phase, no deadline
// miss, no e-stop) and isAttached() (a Driver Station client owns the robot). outputsEnabled()
// alone does not check the link; isAttached() goes false when core releases the DS owner slot
// (no DS frame for PROBOT_DS_OWNER_TIMEOUT_MS, 5 s by default), the same moment the control word
// reports the robot disabled. The output gate
// is re-evaluated on core's user task (pre-cycle) and every 10 ms by an esp_timer. On an emergency
// stop every HAL-owned output pin (SetOutputPin: PWM, DIO outputs, LED data lines, pulse TX) is
// routed to plain GPIO and driven low, lock-free.

#pragma once

#include <stdint.h>

#include <atomic>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include <prosdk/cxx/expected.hpp>

#include "prosdk/hal/Types.h"
#include "prosdk/hal/handles/DigitalHandleResource.hpp"
#include "prosdk/hal/handles/HandlesInternal.hpp"
#include "prosdk/platform/Console.hpp"

namespace prosdk::hal::esp32 {

// ---- capacities (ESP32-S3) ----
inline constexpr int32_t kNumSmartIo = 49;            // GPIO0..48 (22..25 do not exist)
inline constexpr int32_t kNumAnalogInputs = 11;       // channel = GPIO 1..10 (ADC1); 0 invalid
inline constexpr int32_t kNumPwmOutputs = 12;         // MCPWM: 2 groups x 3 timers x 2 generators
inline constexpr int32_t kNumPulseCounters = 4;       // PCNT units (encoders + counters)
inline constexpr int32_t kNumDutyCycleInputs = 6;     // MCPWM capture: 2 groups x 3 channels
inline constexpr int32_t kNumDigitalPwmOutputs = 6;   // LEDC channels (of 8)
// RMT TX channels (of 4; core's status LED uses one), shared by LED outputs and pulse TX
inline constexpr int32_t kNumRmtTxChannels = 3;
inline constexpr int32_t kNumAddressableLeds = kNumRmtTxChannels;  // while no pulse TX is open
inline constexpr int32_t kNumPulseRx = 4;             // RMT RX channels (S3 channels 4-7)
inline constexpr int32_t kNumI2CBuses = 2;
inline constexpr int32_t kNumCanBuses = 1;            // TWAI
inline constexpr int32_t kMaxPwmPulseUs = 4096;       // [0, 4096) like systemcore, or 0xFFFF
inline constexpr int32_t kPwmDisabled = 0;
inline constexpr int32_t kPwmAlwaysHigh = 0xFFFF;

// ---- pin ownership ----

/** What a GPIO is used for (SmartIo::mode). */
enum class PinMode : uint8_t {
  kNone,
  kDigitalInput,
  kDigitalOutput,
  kPwm,
  kCounter,
  kEncoder,
  kDutyCycle,
  kAnalogInput,
  kAddressableLed,
  kPulseTx,     // RMT TX pulse train (PulseIO.cpp)
  kPulseRx,     // RMT RX pulse capture (PulseIO.cpp)
  kPeripheral,  // claimed by a UART or the TWAI controller (ClaimPin)
};

/** Hardware owned by one pin; the destructor releases it (called when the handle is freed). */
struct PinDriver {
  virtual ~PinDriver() = default;
};

/** One GPIO allocated through the HAL. */
struct SmartIo {
  uint8_t channel = 0;
  PinMode mode = PinMode::kNone;
  std::string previousAllocation;
  std::unique_ptr<PinDriver> driver;
};

/** GPIO-indexed handle table shared by DIO, PWM, Counter, Encoder, DutyCycle, Analog, LED,
 * pulse I/O. */
extern DigitalHandleResource<HAL_DigitalHandle, SmartIo, kNumSmartIo>* smartIoHandles;

/** True for a GPIO that exists on the ESP32-S3 (0..21, 26..48). */
bool IsValidGpio(int32_t gpio);

/**
 * Checks that a GPIO may be used by the HAL. Returns HAL_SUCCESS or an error status (with
 * HAL_SetLastError-style message through MakeError) naming why: out of range, not a GPIO,
 * reserved by flash/PSRAM, USB, the UART0 console, probot-core, or another Arduino driver.
 */
HAL_Status CheckPinUsable(int32_t gpio, std::string_view what);

/**
 * Allocates a GPIO in smartIoHandles after CheckPinUsable. `what` names the user ("PWM").
 * The previous-allocation string is set by the caller once initialization succeeded.
 */
prosdk::cxx::expected<std::pair<HAL_DigitalHandle, std::shared_ptr<SmartIo>>, HAL_Status>
AllocateSmartIo(int32_t gpio, HAL_HandleEnum type, std::string_view what);

/** Frees a smartIo handle and waits (up to 1 s) until no other thread holds the port. */
void FreeSmartIo(HAL_DigitalHandle handle, HAL_HandleEnum type,
                 std::shared_ptr<SmartIo>&& port, const char* what);

/**
 * Claims a GPIO for a peripheral that is not one of the pin APIs (UART, TWAI) after
 * CheckPinUsable, so DIO/PWM/... on the same GPIO fail with HAL_RESOURCE_IS_ALLOCATED naming
 * `owner`. `type` is the peripheral's handle type (SERIAL_PORT, CAN). Release with ReleasePin.
 */
prosdk::cxx::expected<HAL_DigitalHandle, HAL_Status> ClaimPin(int32_t gpio, HAL_HandleEnum type,
                                                            std::string_view what,
                                                            std::string_view owner);

/** Releases a ClaimPin claim (no-op for HAL_INVALID_HANDLE). */
void ReleasePin(HAL_DigitalHandle handle, HAL_HandleEnum type);

/** Detaches any Arduino peripheral-manager bus from the pin (no-op when unused). */
void ReleaseArduinoPin(int32_t gpio);

// ---- output safety ----

/** Marks a GPIO as a HAL output that the emergency stop must drive low (or unmarks it). */
void SetOutputPin(int32_t gpio, bool isOutput);

/** True after probot-core reported an emergency stop (latched until reboot). */
bool EStopped();

/** Current output gate: probot-core's outputsEnabled() and isAttached(), not e-stopped. */
bool OutputsEnabled();

/** Re-evaluates the gate now and applies a change to the PWM outputs and the digital PWM
 * generators (task context only). */
void UpdateOutputGate();

// ---- shared RMT TX budget ----
namespace rmt {
/** RMT TX channels taken by HAL users (AddressableLED outputs and pulse transmitters). */
inline constinit std::atomic<int32_t> txChannelsInUse{0};

/**
 * Reserves one of the kNumRmtTxChannels RMT TX channels that AddressableLED and pulse TX share
 * (the 4th belongs to probot-core's status LED). Returns false when all are taken. Call before
 * rmt_new_tx_channel; release with ReleaseTxChannel when the channel is deleted.
 */
inline bool AcquireTxChannel() {
  int32_t n = txChannelsInUse.load(std::memory_order_relaxed);
  do {
    if (n >= kNumRmtTxChannels) {
      return false;
    }
  } while (!txChannelsInUse.compare_exchange_weak(n, n + 1, std::memory_order_acq_rel));
  return true;
}

/** Returns a channel taken by AcquireTxChannel. */
inline void ReleaseTxChannel() {
  txChannelsInUse.fetch_sub(1, std::memory_order_acq_rel);
}
}  // namespace rmt

// ---- per-module hooks (defined in the module .cpp files) ----
namespace pwm {
/** Applies the output gate to every PWM output. Returns false if it could not lock (retry). */
bool ApplyGate(bool enabled, bool blocking);
}  // namespace pwm

namespace dio {
/** Applies the output gate to every digital PWM generator (duty 0 while closed). Returns false
 * if it could not lock (retry). */
bool ApplyGate(bool enabled, bool blocking);
}  // namespace dio

namespace can {
/** Lock-free: stops periodic CAN transmissions after an emergency stop. */
void OnEmergencyStop();
}  // namespace can

namespace sixstep {
/** Applies the output gate to every six-step motor (coast while closed; SixStep.cpp). Returns
 * false if it could not lock (retry). */
bool ApplyGate(bool enabled, bool blocking);
/** Coasts every six-step motor for good: all 6 outputs low, under the Hall interrupt's spinlock
 * (register writes only). */
void OnEmergencyStop();
}  // namespace sixstep

/**
 * Logs one diagnostic line through the prosdk console (adds the "[prosdk hal] " prefix). HAL
 * errors are console errors; degraded but working features pass kWarning.
 */
void Log(std::string_view text,
         platform::ConsoleLevel level = platform::ConsoleLevel::kError);

/** esp_err_t to a short name for messages. */
const char* EspErrName(int err);

}  // namespace prosdk::hal::esp32

namespace prosdk::hal::init {
extern std::atomic_bool HAL_IsInitialized;
void RunInitialize();
inline void CheckInit() {
  if (HAL_IsInitialized.load(std::memory_order_relaxed)) {
    return;
  }
  RunInitialize();
}

void InitializeAddressableLED();
void InitializeAnalogInput();
void InitializeCAN();
void InitializeCANAPI();
void InitializeCounter();
void InitializeDIO();
void InitializeDutyCycle();
void InitializeEncoder();
void InitializeI2C();
void InitializeMain();
void InitializeNotifier();
void InitializePWM();
void InitializeSerialPort();
void InitializeSmartIo();
void InitializeDriverStationPump();
}  // namespace prosdk::hal::init
