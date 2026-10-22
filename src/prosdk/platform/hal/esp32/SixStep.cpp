// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: six-step BLDC commutation from 3 Hall sensors
// (prosdk/platform/hal/SixStep.hpp, design/boardoza/P15-drv8317h.md).
//
// PWM: one MCPWM timer (10 MHz, up-counting) with one operator, comparator and generator per
// motor: high at timer zero, low at the compare value. The generator is created on INHA, its
// GPIO-matrix output selection (GPIO_FUNCn_OUT_SEL_CFG) is read back, and INHA is returned to
// the plain GPIO output. From then on the commutation routes that one PWM signal to the INHx pin
// of the phase that switches and holds the other INHx pins on the plain GPIO output, which stays
// low. In 3x PWM mode only one phase switches per step (SixStep.hpp), so one generator is enough.
// Cost: one of the 6 MCPWM timer/operator pairs the servo PWM outputs use (2 of their 12
// outputs). LEDC is not used: its 6 channels and one shared rate belong to the digital PWM
// generators (DIO.cpp).
//
// Commutation: a GPIO interrupt on any edge of each Hall pin (the GPIO ISR service, shared with
// Arduino's attachInterrupt and installed with the same flags, ARDUINO_ISR_FLAG). The handler
// and everything it touches are in IRAM and internal RAM: 3 Hall reads, a lookup in a table
// precomputed per mode and Hall state, up to 6 GPIO register writes and esp_timer_get_time for
// the step clock. The ISR service itself is IRAM-safe only when Arduino-ESP32 is built with
// CONFIG_ARDUINO_ISR_IRAM (off in 3.3.10): otherwise Hall interrupts wait while the flash cache
// is disabled (flash writes such as LittleFS logs) and the last step stays applied meanwhile.
//
// Switching order per step: phases that float lose INLx first, then the INHx routes change,
// then the driven phases get INLx; every intermediate state is Hi-Z or a subset of the target.
//
// Safety: all 6 outputs are HAL output pins (SetOutputPin). While the output gate is closed the
// motor coasts (all INLx low). sixstep::OnEmergencyStop (called by SmartIo.cpp before the generic
// pin loop) coasts every motor under the same spinlock as the interrupt and latches a flag, so no
// interrupt or task can drive a phase afterwards; the generic loop then routes all 6 pins to the
// plain GPIO output, low.
//
// Unverified on hardware (design/boardoza/P15-drv8317h.md): interrupt latency, behaviour under
// Hall noise at PWM edges, the MCPWM group clock accepting 10 MHz next to the 1 MHz servo timers,
// and that copying the generator's matrix selection to another pin carries the PWM.

#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>
#include <esp32-hal.h>
#include <esp_attr.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_struct.h>

#include <atomic>
#include <cmath>
#include <format>
#include <mutex>

#include "Esp32Internal.hpp"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/platform/hal/SixStep.hpp"

using namespace prosdk::hal;
using namespace prosdk::hal::esp32;
using namespace prosdk::platform::hal;
namespace six = prosdk::platform::hal::detail;
using six::SixStepMode;

namespace {

constexpr int kNumGroups = 2;
constexpr int kNumModes = 4;  // SixStepMode values
constexpr const char* kEStopped =
    "six-step: the robot is emergency stopped; outputs stay off until reboot";

struct Motor {
  // ---- used by the Hall interrupt (gLock) ----
  bool open = false;
  uint8_t inh[3] = {};
  uint8_t inl[3] = {};
  uint8_t hall[3] = {};
  uint32_t pwmSel = SIG_GPIO_OUT_IDX;   // matrix selection that carries the PWM
  uint8_t packed[kNumModes][8] = {};    // 2 bits per phase (SixStepPhase), per mode and state
  int8_t step[8] = {};                  // table step index per Hall state, -1 invalid
  uint8_t mode = 0;                     // effective SixStepMode
  uint8_t applied = 0xFF;               // packed phases on the pins (0xFF: unknown)
  int32_t hallState = 0;
  int8_t lastStep = -1;
  int64_t steps = 0;
  int64_t lastStepUs = 0;
  int64_t periodUs = 0;
  uint32_t badStates = 0;

  // ---- task side (gMutex) ----
  mcpwm_timer_handle_t timer = nullptr;
  mcpwm_oper_handle_t oper = nullptr;
  mcpwm_cmpr_handle_t cmpr = nullptr;
  mcpwm_gen_handle_t gen = nullptr;
  uint32_t periodTicks = 0;
  SixStepMode requestedMode = SixStepMode::kCoast;
  uint32_t requestedTicks = 0;
  HAL_DigitalHandle claims[9] = {HAL_INVALID_HANDLE, HAL_INVALID_HANDLE, HAL_INVALID_HANDLE,
                                 HAL_INVALID_HANDLE, HAL_INVALID_HANDLE, HAL_INVALID_HANDLE,
                                 HAL_INVALID_HANDLE, HAL_INVALID_HANDLE, HAL_INVALID_HANDLE};
  bool pinsReady = false;  // all 9 pins claimed and inh/inl/hall set
  bool handlers = false;   // Hall interrupt handlers added
};

// gLock serializes the Hall interrupt, the task-side output changes and the emergency stop.
// The motors are static (internal RAM, never freed): a late interrupt never touches freed memory.
portMUX_TYPE gLock = portMUX_INITIALIZER_UNLOCKED;
std::mutex gMutex;
Motor gMotors[kMaxSixStepMotors];
bool gStopped = false;    // set by OnEmergencyStop under gLock, never cleared
bool gGate = false;       // output gate last applied (gMutex)
bool gIsrService = false; // GPIO ISR service installed (gMutex)

uint8_t Pack(const std::array<SixStepPhase, 3>& phases) {
  return static_cast<uint8_t>(static_cast<uint8_t>(phases[0]) |
                              (static_cast<uint8_t>(phases[1]) << 2) |
                              (static_cast<uint8_t>(phases[2]) << 4));
}

void Precompute(Motor& m, const SixStepTable& table) {
  for (int state = 0; state < 8; ++state) {
    m.step[state] = static_cast<int8_t>(six::StepOf(table[state]));
    for (int mode = 0; mode < kNumModes; ++mode) {
      m.packed[mode][state] =
          Pack(six::PhasesFor(table, state, static_cast<SixStepMode>(mode)));
    }
  }
}

// ---- interrupt-safe part (IRAM, register access only) ----

IRAM_ATTR int ReadHalls(const Motor& m) {
  int state = 0;
  for (int i = 0; i < 3; ++i) {
    state |= gpio_ll_get_level(&GPIO, m.hall[i]) << i;
  }
  return state;
}

// gLock held. Puts the pins in the packed phase state.
IRAM_ATTR void Drive(Motor& m, uint8_t packed) {
  if (packed == m.applied) {
    return;
  }
  for (int i = 0; i < 3; ++i) {
    if (((packed >> (2 * i)) & 3) == static_cast<uint8_t>(SixStepPhase::kFloat)) {
      gpio_ll_set_level(&GPIO, m.inl[i], 0);
    }
  }
  for (int i = 0; i < 3; ++i) {
    bool pwm = ((packed >> (2 * i)) & 3) == static_cast<uint8_t>(SixStepPhase::kPwm);
    REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * m.inh[i], pwm ? m.pwmSel : SIG_GPIO_OUT_IDX);
  }
  for (int i = 0; i < 3; ++i) {
    if (((packed >> (2 * i)) & 3) != static_cast<uint8_t>(SixStepPhase::kFloat)) {
      gpio_ll_set_level(&GPIO, m.inl[i], 1);
    }
  }
  m.applied = packed;
}

// gLock held. Counts the step to a new Hall state, then commutates.
IRAM_ATTR void OnHallState(Motor& m, int state) {
  int64_t now = esp_timer_get_time();
  int step = m.step[state];
  if (step < 0) {
    ++m.badStates;
  } else if (m.lastStep >= 0 && step != m.lastStep) {
    int d = step - m.lastStep;
    if (d < 0) {
      d += 6;
    }
    if (d == 3) {
      ++m.badStates;  // 3 steps: the direction is unknown
    } else {
      int delta = d <= 2 ? d : d - 6;
      int64_t dt = now - m.lastStepUs;
      if (delta == 2 || delta == -2) {
        dt >>= 1;  // a step was missed: the period is half the time
      }
      m.periodUs = delta > 0 ? dt : -dt;
      m.steps += delta;
      m.lastStepUs = now;
    }
  }
  if (step >= 0) {
    if (m.lastStep < 0) {
      m.lastStepUs = now;
    }
    m.lastStep = static_cast<int8_t>(step);
  }
  m.hallState = state;
  Drive(m, gStopped ? 0 : m.packed[m.mode][state]);
}

IRAM_ATTR void HallIsr(void* arg) {
  Motor& m = *static_cast<Motor*>(arg);
  portENTER_CRITICAL_ISR(&gLock);
  if (m.open) {
    int state = ReadHalls(m);
    if (state != m.hallState) {
      OnHallState(m, state);
    }
  }
  portEXIT_CRITICAL_ISR(&gLock);
}

// ---- task side ----

Motor* Find(int32_t handle) {
  int slot = handle - 1;
  return slot >= 0 && slot < kMaxSixStepMotors && gMotors[slot].open ? &gMotors[slot] : nullptr;
}

// gMutex held. Applies the gate and the request: comparator first, then the mode and the pins.
// Every call also re-reads the Halls, which recovers a missed edge at standstill.
void ApplyMotorLocked(Motor& m) {
  SixStepMode mode = gGate ? m.requestedMode : SixStepMode::kCoast;
  if ((mode == SixStepMode::kForward || mode == SixStepMode::kReverse) && m.cmpr != nullptr) {
    mcpwm_comparator_set_compare_value(m.cmpr, m.requestedTicks);
  }
  portENTER_CRITICAL(&gLock);
  m.mode = static_cast<uint8_t>(gStopped ? SixStepMode::kCoast : mode);
  int state = ReadHalls(m);
  if (state != m.hallState) {
    OnHallState(m, state);
  } else {
    Drive(m, gStopped ? 0 : m.packed[m.mode][state]);
  }
  portEXIT_CRITICAL(&gLock);
}

// gMutex held.
void ApplyGateLocked(bool enabled) {
  if (enabled == gGate) {
    return;
  }
  gGate = enabled;
  for (Motor& m : gMotors) {
    if (m.open) {
      ApplyMotorLocked(m);
    }
  }
}

// Routes a pin back to the plain GPIO output, driven low (same as the emergency stop).
void ForceLow(int gpio) {
  gpio_set_level(static_cast<gpio_num_t>(gpio), 0);
  REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * gpio, SIG_GPIO_OUT_IDX);
}

// gMutex held. Releases everything a (possibly half-opened) motor holds. The outputs coast
// first, which also routes every INHx to the plain GPIO output, so the MCPWM objects can go
// before the pins (deleting the generator may disable its pad's output; the pins are set up
// afterwards). Released output pins get a pull-down: gpio_reset_pin's default pull-up would read
// as INLx = INHx = 1 at the driver, high side on in 3x PWM mode. After an emergency stop they
// stay driven low instead.
void Release(Motor& m) {
  if (m.handlers) {
    for (uint8_t pin : m.hall) {
      gpio_intr_disable(static_cast<gpio_num_t>(pin));
      gpio_isr_handler_remove(static_cast<gpio_num_t>(pin));
    }
  }
  portENTER_CRITICAL(&gLock);
  m.open = false;
  if (m.pinsReady) {
    m.applied = 0xFF;
    Drive(m, 0);
  }
  portEXIT_CRITICAL(&gLock);
  if (m.gen != nullptr) {
    mcpwm_generator_set_force_level(m.gen, 0, true);
    mcpwm_del_generator(m.gen);
  }
  if (m.cmpr != nullptr) {
    mcpwm_del_comparator(m.cmpr);
  }
  if (m.oper != nullptr) {
    mcpwm_del_operator(m.oper);
  }
  if (m.timer != nullptr) {
    mcpwm_timer_start_stop(m.timer, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(m.timer);
    mcpwm_del_timer(m.timer);
  }
  bool stopped = EStopped();
  for (int i = 0; m.pinsReady && i < 6; ++i) {
    int gpio = i < 3 ? m.inh[i] : m.inl[i - 3];
    auto pin = static_cast<gpio_num_t>(gpio);
    if (stopped) {
      ForceLow(gpio);
      gpio_set_direction(pin, GPIO_MODE_OUTPUT);  // the pad may have lost its output enable
    } else {
      SetOutputPin(gpio, false);
      gpio_reset_pin(pin);
      gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY);
    }
  }
  for (int i = 0; m.pinsReady && i < 3; ++i) {
    gpio_reset_pin(static_cast<gpio_num_t>(m.hall[i]));
  }
  for (HAL_DigitalHandle claim : m.claims) {
    ReleasePin(claim, HAL_HandleEnum::VENDOR);
  }
  portENTER_CRITICAL(&gLock);
  m = Motor{};
  portEXIT_CRITICAL(&gLock);
}

// gMutex held. The MCPWM timer, operator, comparator and generator (on INHA), forced low.
HAL_Status StartPwm(Motor& m, uint32_t pwmHz) {
  m.periodTicks = kSixStepResolutionHz / pwmHz;
  esp_err_t err = ESP_FAIL;
  for (int group = 0; group < kNumGroups && m.oper == nullptr; ++group) {
    mcpwm_timer_config_t tc{};
    tc.group_id = group;
    tc.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    tc.resolution_hz = kSixStepResolutionHz;
    tc.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    tc.period_ticks = m.periodTicks;
    err = mcpwm_new_timer(&tc, &m.timer);
    if (err != ESP_OK) {
      m.timer = nullptr;
      continue;
    }
    mcpwm_operator_config_t oc{};
    oc.group_id = group;
    err = mcpwm_new_operator(&oc, &m.oper);
    if (err != ESP_OK) {
      m.oper = nullptr;
      mcpwm_del_timer(m.timer);
      m.timer = nullptr;
    }
  }
  if (m.oper == nullptr) {
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     std::format("six-step: no MCPWM timer and operator left ({}; the servo PWM "
                                 "outputs use the same 6)",
                                 EspErrName(err)));
  }
  err = mcpwm_operator_connect_timer(m.oper, m.timer);
  if (err == ESP_OK) {
    mcpwm_comparator_config_t cc{};
    cc.flags.update_cmp_on_tez = 1;
    err = mcpwm_new_comparator(m.oper, &cc, &m.cmpr);
    if (err != ESP_OK) {
      m.cmpr = nullptr;
    }
  }
  if (err == ESP_OK) {
    mcpwm_generator_config_t gc{};
    gc.gen_gpio_num = m.inh[0];
    err = mcpwm_new_generator(m.oper, &gc, &m.gen);
    if (err != ESP_OK) {
      m.gen = nullptr;
    }
  }
  if (err == ESP_OK) {
    mcpwm_generator_set_force_level(m.gen, 0, true);
    // the selection that carries this generator's PWM, then INHA back to the GPIO output (low)
    m.pwmSel = REG_READ(GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * m.inh[0]);
    ForceLow(m.inh[0]);
    if ((m.pwmSel & 0x1FF) == SIG_GPIO_OUT_IDX) {
      err = ESP_ERR_NOT_SUPPORTED;  // the generator did not take the pin through the matrix
    }
  }
  if (err == ESP_OK) {
    mcpwm_gen_timer_event_action_t onEmpty{};
    onEmpty.direction = MCPWM_TIMER_DIRECTION_UP;
    onEmpty.event = MCPWM_TIMER_EVENT_EMPTY;
    onEmpty.action = MCPWM_GEN_ACTION_HIGH;
    mcpwm_gen_compare_event_action_t onCompare{};
    onCompare.direction = MCPWM_TIMER_DIRECTION_UP;
    onCompare.comparator = m.cmpr;
    onCompare.action = MCPWM_GEN_ACTION_LOW;
    err = mcpwm_generator_set_action_on_timer_event(m.gen, onEmpty);
    if (err == ESP_OK) {
      err = mcpwm_generator_set_action_on_compare_event(m.gen, onCompare);
    }
    if (err == ESP_OK) {
      err = mcpwm_comparator_set_compare_value(m.cmpr, 0);
    }
    if (err == ESP_OK) {
      err = mcpwm_timer_enable(m.timer);
    }
    if (err == ESP_OK) {
      err = mcpwm_timer_start_stop(m.timer, MCPWM_TIMER_START_NO_STOP);
    }
    if (err == ESP_OK) {
      err = mcpwm_generator_set_force_level(m.gen, -1, true);
    }
  }
  if (err != ESP_OK) {
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     std::format("six-step: MCPWM setup failed on GPIO {} ({})", m.inh[0],
                                 EspErrName(err)));
  }
  return HAL_SUCCESS;
}

// gMutex held. Claims and configures the 9 pins: outputs low, Halls inputs with pull-up.
HAL_Status SetUpPins(Motor& m, const SixStepPins& pins, const char* owner) {
  const int32_t all[9] = {pins.inh[0], pins.inh[1], pins.inh[2], pins.inl[0], pins.inl[1],
                          pins.inl[2], pins.hall[0], pins.hall[1], pins.hall[2]};
  for (int i = 0; i < 9; ++i) {
    auto claim = ClaimPin(all[i], HAL_HandleEnum::VENDOR, "Six-step", owner);
    if (!claim) {
      return claim.error();
    }
    m.claims[i] = *claim;
  }
  for (int i = 0; i < 3; ++i) {
    m.inh[i] = static_cast<uint8_t>(pins.inh[i]);
    m.inl[i] = static_cast<uint8_t>(pins.inl[i]);
    m.hall[i] = static_cast<uint8_t>(pins.hall[i]);
  }
  m.pinsReady = true;
  for (int i = 0; i < 6; ++i) {
    auto pin = static_cast<gpio_num_t>(all[i]);
    gpio_set_level(pin, 0);  // before the pad turns into an output: no high glitch
    gpio_config_t out{};
    out.pin_bit_mask = uint64_t{1} << all[i];
    out.mode = GPIO_MODE_OUTPUT;
    out.intr_type = GPIO_INTR_DISABLE;
    if (gpio_config(&out) != ESP_OK) {
      return MakeError(HAL_RESOURCE_IS_ALLOCATED,
                       std::format("six-step: GPIO {} could not be made an output", all[i]));
    }
    ForceLow(all[i]);
    SetOutputPin(all[i], true);  // the emergency stop drives it low
  }
  for (int i = 6; i < 9; ++i) {
    gpio_config_t in{};
    in.pin_bit_mask = uint64_t{1} << all[i];
    in.mode = GPIO_MODE_INPUT;
    in.pull_up_en = GPIO_PULLUP_ENABLE;
    in.intr_type = GPIO_INTR_DISABLE;
    if (gpio_config(&in) != ESP_OK) {
      return MakeError(HAL_RESOURCE_IS_ALLOCATED,
                       std::format("six-step: GPIO {} could not be made an input", all[i]));
    }
  }
  return HAL_SUCCESS;
}

// gMutex held. Hall interrupts on any edge.
HAL_Status StartHalls(Motor& m) {
  if (!gIsrService) {
    // the flags Arduino's attachInterrupt uses, so both can share the service
    esp_err_t err = gpio_install_isr_service(ARDUINO_ISR_FLAG);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                       std::format("six-step: GPIO interrupt service failed ({})",
                                   EspErrName(err)));
    }
    gIsrService = true;
  }
  for (uint8_t pin : m.hall) {
    auto gpio = static_cast<gpio_num_t>(pin);
    esp_err_t err = gpio_isr_handler_add(gpio, &HallIsr, &m);
    m.handlers = true;
    if (err == ESP_OK) {
      err = gpio_set_intr_type(gpio, GPIO_INTR_ANYEDGE);
    }
    if (err == ESP_OK) {
      err = gpio_intr_enable(gpio);
    }
    if (err != ESP_OK) {
      return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                       std::format("six-step: Hall interrupt on GPIO {} failed ({})", pin,
                                   EspErrName(err)));
    }
  }
  return HAL_SUCCESS;
}

}  // namespace

namespace prosdk::hal::esp32::sixstep {

bool ApplyGate(bool enabled, bool blocking) {
  std::unique_lock lock(gMutex, std::defer_lock);
  if (blocking) {
    lock.lock();
  } else if (!lock.try_lock()) {
    return false;
  }
  ApplyGateLocked(enabled);
  return true;
}

void OnEmergencyStop() {
  portENTER_CRITICAL(&gLock);
  gStopped = true;
  for (Motor& m : gMotors) {
    if (m.open) {
      m.mode = static_cast<uint8_t>(SixStepMode::kCoast);
      m.applied = 0xFF;  // write all six pins
      Drive(m, 0);
    }
  }
  portEXIT_CRITICAL(&gLock);
}

}  // namespace prosdk::hal::esp32::sixstep

namespace prosdk::platform::hal {

int32_t OpenSixStep(const SixStepPins& pins, uint32_t pwmHz, const char* allocationLocation,
                    int32_t* status) {
  prosdk::hal::init::CheckInit();
  if (pwmHz < kMinSixStepPwmHz || pwmHz > kMaxSixStepPwmHz) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE,
                        "six-step: the PWM frequency must be 1 kHz to 100 kHz");
    return kInvalidSixStepHandle;
  }
  if (EStopped()) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE, kEStopped);
    return kInvalidSixStepHandle;
  }
  bool enabled = OutputsEnabled();
  std::scoped_lock lock(gMutex);
  int slot = -1;
  for (int i = 0; i < kMaxSixStepMotors; ++i) {
    if (!gMotors[i].open && gMotors[i].claims[0] == HAL_INVALID_HANDLE) {
      slot = i;
      break;
    }
  }
  if (slot < 0) {
    *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                        std::format("six-step: all {} motor slots are in use",
                                    kMaxSixStepMotors));
    return kInvalidSixStepHandle;
  }
  Motor& m = gMotors[slot];
  Precompute(m, kDefaultSixStepTable);
  *status = SetUpPins(m, pins, allocationLocation ? allocationLocation : "six-step motor");
  if (*status == HAL_SUCCESS) {
    *status = StartPwm(m, pwmHz);
  }
  if (*status == HAL_SUCCESS) {
    portENTER_CRITICAL(&gLock);
    m.mode = static_cast<uint8_t>(SixStepMode::kCoast);
    m.hallState = ReadHalls(m);
    m.lastStep = m.step[m.hallState];
    m.lastStepUs = esp_timer_get_time();
    m.applied = 0xFF;
    Drive(m, 0);
    m.open = true;
    portEXIT_CRITICAL(&gLock);
    *status = StartHalls(m);
  }
  if (*status == HAL_SUCCESS && EStopped()) {
    // the emergency stop ran while the motor was being set up
    *status = MakeError(HAL_INCOMPATIBLE_STATE, kEStopped);
  }
  if (*status != HAL_SUCCESS) {
    Release(m);
    return kInvalidSixStepHandle;
  }
  ApplyGateLocked(enabled);
  return slot + 1;
}

void SetSixStepTable(int32_t handle, const SixStepTable& table, int32_t* status) {
  if (const char* error = six::CheckSixStepTable(table)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE, error);
    return;
  }
  std::scoped_lock lock(gMutex);
  Motor* m = Find(handle);
  if (m == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  Motor next = *m;  // the new tables, swapped in under gLock
  Precompute(next, table);
  portENTER_CRITICAL(&gLock);
  for (int state = 0; state < 8; ++state) {
    m->step[state] = next.step[state];
    for (int mode = 0; mode < kNumModes; ++mode) {
      m->packed[mode][state] = next.packed[mode][state];
    }
  }
  m->lastStep = m->step[m->hallState];
  portEXIT_CRITICAL(&gLock);
  ApplyMotorLocked(*m);
  *status = HAL_SUCCESS;
}

void SetSixStepOutput(int32_t handle, double duty, bool brake, int32_t* status) {
  bool enabled = OutputsEnabled();
  std::scoped_lock lock(gMutex);
  Motor* m = Find(handle);
  if (m == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  *status = HAL_SUCCESS;
  if (EStopped()) {
    return;  // ignored: the motor coasts until reboot
  }
  if (std::isnan(duty)) {
    duty = 0.0;
  }
  m->requestedTicks = six::DutyTicks(duty, m->periodTicks);
  if (m->requestedTicks == 0) {
    m->requestedMode = brake ? SixStepMode::kBrake : SixStepMode::kCoast;
  } else {
    m->requestedMode = duty > 0.0 ? SixStepMode::kForward : SixStepMode::kReverse;
  }
  if (enabled != gGate) {
    ApplyGateLocked(enabled);  // applies this motor too
  } else {
    ApplyMotorLocked(*m);
  }
}

bool GetSixStepFeedback(int32_t handle, SixStepFeedback* out, int32_t* status) {
  std::scoped_lock lock(gMutex);
  Motor* m = Find(handle);
  if (m == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }
  uint8_t applied;
  portENTER_CRITICAL(&gLock);
  out->hallState = m->hallState;
  out->steps = m->steps;
  out->lastStepUs = m->lastStepUs;
  out->periodUs = m->periodUs;
  out->badStates = m->badStates;
  applied = m->applied == 0xFF ? 0 : m->applied;
  portEXIT_CRITICAL(&gLock);
  out->nowUs = esp_timer_get_time();
  for (int i = 0; i < 3; ++i) {
    out->phases[i] = static_cast<SixStepPhase>((applied >> (2 * i)) & 3);
  }
  *status = HAL_SUCCESS;
  return true;
}

void CloseSixStep(int32_t handle) {
  std::scoped_lock lock(gMutex);
  Motor* m = Find(handle);
  if (m != nullptr) {
    Release(*m);
  }
}

}  // namespace prosdk::platform::hal
