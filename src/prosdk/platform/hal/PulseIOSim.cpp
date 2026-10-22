// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Host simulation of the pulse I/O backend (prosdk/platform/hal/PulseIO.hpp) and its test hooks
// (prosdk/simulation/PulseIOSim.hpp). It applies the ESP32-S3 backend's argument checks and
// channel counts (3 transmitters, 4 captures; LED outputs are not counted here) and one owner per
// GPIO among the pulse channels; it does not check other HAL users of the pin. Trains are
// "sent" at once (WaitPulsesSent is always true) and there is no emergency stop. Compiled into
// both builds by the platform/hal glob; empty on the robot.

#include "prosdk/platform/config.hpp"

#if !PROSDK_TARGET_HW

#include <deque>
#include <format>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/platform/hal/PulseIO.hpp"
#include "prosdk/simulation/PulseIOSim.hpp"

namespace {

using prosdk::hal::MakeError;
using prosdk::platform::hal::kInvalidPulseHandle;
using prosdk::platform::hal::Pulse;

constexpr int kNumGpio = 49;  // GPIO 0..48, as the ESP32-S3 backend
constexpr int kNumTx = 3;     // RMT TX channels left for the HAL
constexpr int kNumRx = 4;     // RMT RX channels
constexpr int32_t kRxHandleBase = 0x100;  // TX handle = slot + 1, RX handle = base + slot

struct Tx {
  bool open = false;
  int gpio = -1;
  uint32_t carrierHz = 0;
  double carrierDuty = 0.0;
  std::string allocation;
};

struct Rx {
  bool open = false;
  int gpio = -1;
  uint32_t glitchUs = 0;
  uint32_t idleUs = 0;
  std::string allocation;
  std::deque<Pulse> buffer;
  int64_t dropped = 0;
  bool inBurst = false;      // a burst started and has not ended
  bool idleKnown = false;    // the line idles at idleLevel since the last burst end
  bool idleLevel = false;
};

struct Callback {
  int gpio;
  prosdk::sim::PulsesSentFn fn;
};

// recursive: a sent-callback may inject pulses into a capture (loopback)
std::recursive_mutex gMutex;
Tx gTx[kNumTx];
Rx gRx[kNumRx];
std::map<int, Callback> gCallbacks;
int gNextId = 1;

Tx* FindTx(int32_t handle) {
  int slot = handle - 1;
  return slot >= 0 && slot < kNumTx && gTx[slot].open ? &gTx[slot] : nullptr;
}

Rx* FindRx(int32_t handle) {
  int slot = handle - kRxHandleBase;
  return slot >= 0 && slot < kNumRx && gRx[slot].open ? &gRx[slot] : nullptr;
}

Rx* FindRxByGpio(int gpio) {
  for (Rx& rx : gRx) {
    if (rx.open && rx.gpio == gpio) {
      return &rx;
    }
  }
  return nullptr;
}

// gMutex held. HAL_SUCCESS when `gpio` is a GPIO no pulse channel uses.
int32_t CheckGpio(int32_t gpio, const char* what) {
  if (gpio < 0 || gpio >= kNumGpio) {
    return prosdk::hal::MakeErrorIndexOutOfRange(HAL_RESOURCE_OUT_OF_RANGE,
                                                 std::format("Invalid Index for {}", what), 0,
                                                 kNumGpio - 1, gpio);
  }
  for (const Tx& tx : gTx) {
    if (tx.open && tx.gpio == gpio) {
      return prosdk::hal::MakeErrorPreviouslyAllocated(HAL_RESOURCE_IS_ALLOCATED, what, gpio,
                                                       tx.allocation);
    }
  }
  for (const Rx& rx : gRx) {
    if (rx.open && rx.gpio == gpio) {
      return prosdk::hal::MakeErrorPreviouslyAllocated(HAL_RESOURCE_IS_ALLOCATED, what, gpio,
                                                       rx.allocation);
    }
  }
  return HAL_SUCCESS;
}

// gMutex held. One captured pulse into the buffer (dropped when it is full).
void Store(Rx& rx, Pulse p) {
  if (rx.buffer.size() >= static_cast<size_t>(prosdk::platform::hal::kRxBufferPulses)) {
    ++rx.dropped;
    return;
  }
  rx.buffer.push_back(p);
}

}  // namespace

namespace prosdk::platform::hal {

int32_t OpenPulseTx(int32_t gpio, uint32_t carrierHz, double carrierDuty,
                    const char* allocationLocation, int32_t* status) {
  if (const char* error = detail::CheckPulseTx(carrierHz, carrierDuty)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE, error);
    return kInvalidPulseHandle;
  }
  std::scoped_lock lock(gMutex);
  *status = CheckGpio(gpio, "Pulse TX");
  if (*status != HAL_SUCCESS) {
    return kInvalidPulseHandle;
  }
  for (int i = 0; i < kNumTx; ++i) {
    if (!gTx[i].open) {
      gTx[i] = Tx{true, gpio, carrierHz, carrierHz != 0 ? carrierDuty : 0.0,
                  allocationLocation ? allocationLocation : ""};
      return i + 1;
    }
  }
  *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                      std::format("pulse TX: all {} RMT TX channels are in use (LED outputs and "
                                  "pulse transmitters share them)",
                                  kNumTx));
  return kInvalidPulseHandle;
}

void SendPulses(int32_t handle, std::span<const Pulse> pulses, int32_t* status) {
  std::scoped_lock lock(gMutex);
  Tx* tx = FindTx(handle);
  if (tx == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  if (const char* error = detail::CheckPulseTrain(pulses)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE, error);
    return;
  }
  *status = HAL_SUCCESS;
  if (pulses.empty()) {
    return;
  }
  // copies: a callback may cancel itself or register another one
  std::vector<prosdk::sim::PulsesSentFn> calls;
  for (const auto& [id, cb] : gCallbacks) {
    if (cb.gpio == tx->gpio) {
      calls.push_back(cb.fn);
    }
  }
  uint32_t carrierHz = tx->carrierHz;
  for (const auto& fn : calls) {
    fn(pulses, carrierHz);
  }
}

bool WaitPulsesSent(int32_t handle, int32_t /*timeoutMs*/, int32_t* status) {
  std::scoped_lock lock(gMutex);
  if (FindTx(handle) == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }
  *status = HAL_SUCCESS;
  return true;
}

int32_t OpenPulseRx(int32_t gpio, uint32_t glitchUs, uint32_t idleUs,
                    const char* allocationLocation, int32_t* status) {
  if (const char* error = detail::CheckPulseRx(glitchUs, idleUs)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE, error);
    return kInvalidPulseHandle;
  }
  std::scoped_lock lock(gMutex);
  *status = CheckGpio(gpio, "Pulse RX");
  if (*status != HAL_SUCCESS) {
    return kInvalidPulseHandle;
  }
  for (int i = 0; i < kNumRx; ++i) {
    if (!gRx[i].open) {
      gRx[i] = Rx{};
      gRx[i].open = true;
      gRx[i].gpio = gpio;
      gRx[i].glitchUs = glitchUs;
      gRx[i].idleUs = idleUs;
      gRx[i].allocation = allocationLocation ? allocationLocation : "";
      return kRxHandleBase + i;
    }
  }
  *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                      std::format("pulse RX: all {} RMT RX channels are in use", kNumRx));
  return kInvalidPulseHandle;
}

int32_t ReadPulses(int32_t handle, std::span<Pulse> out, int32_t* status) {
  std::scoped_lock lock(gMutex);
  Rx* rx = FindRx(handle);
  if (rx == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return 0;
  }
  size_t n = 0;
  while (n < out.size() && !rx->buffer.empty()) {
    out[n++] = rx->buffer.front();
    rx->buffer.pop_front();
  }
  *status = HAL_SUCCESS;
  return static_cast<int32_t>(n);
}

int64_t GetDroppedPulses(int32_t handle, int32_t* status) {
  std::scoped_lock lock(gMutex);
  Rx* rx = FindRx(handle);
  if (rx == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return 0;
  }
  *status = HAL_SUCCESS;
  return rx->dropped;
}

void ClosePulse(int32_t handle) {
  std::scoped_lock lock(gMutex);
  if (Tx* tx = FindTx(handle)) {
    *tx = Tx{};
  } else if (Rx* rx = FindRx(handle)) {
    *rx = Rx{};
  }
}

}  // namespace prosdk::platform::hal

namespace prosdk::sim {

int OnPulsesSent(int gpio, PulsesSentFn fn) {
  std::scoped_lock lock(gMutex);
  int id = gNextId++;
  gCallbacks.emplace(id, Callback{gpio, std::move(fn)});
  return id;
}

void CancelPulsesSent(int id) {
  std::scoped_lock lock(gMutex);
  gCallbacks.erase(id);
}

bool InjectPulses(int gpio, std::span<const Pulse> line) {
  std::scoped_lock lock(gMutex);
  Rx* rx = FindRxByGpio(gpio);
  if (rx == nullptr) {
    return false;
  }
  std::optional<Pulse> pending;  // the current level; its length may still grow
  auto emit = [rx](Pulse p) {
    Store(*rx, p);
    rx->inBurst = true;
  };
  for (const Pulse& p : line) {
    if (p.us == 0) {
      continue;
    }
    if (p.us < rx->glitchUs) {
      // filtered: the line keeps its level, which lasts that much longer
      if (pending) {
        pending->us += p.us;
      }
      continue;
    }
    if (pending) {
      if (pending->level == p.level) {
        pending->us += p.us;
      } else {
        emit(*pending);
        pending = p;
      }
    } else if (!rx->inBurst && rx->idleKnown && p.level == rx->idleLevel) {
      continue;  // the line still idles: no edge, nothing captured
    } else {
      pending = p;
    }
    if (pending->us >= rx->idleUs) {
      if (rx->inBurst) {
        Store(*rx, Pulse{pending->level, 0});
      }
      rx->inBurst = false;
      rx->idleKnown = true;
      rx->idleLevel = pending->level;
      pending.reset();
    }
  }
  if (pending) {
    emit(*pending);
  }
  return true;
}

bool GetPulseTxCarrier(int gpio, uint32_t* carrierHz, double* carrierDuty) {
  std::scoped_lock lock(gMutex);
  for (const Tx& tx : gTx) {
    if (tx.open && tx.gpio == gpio) {
      *carrierHz = tx.carrierHz;
      *carrierDuty = tx.carrierDuty;
      return true;
    }
  }
  return false;
}

}  // namespace prosdk::sim

#endif  // !PROSDK_TARGET_HW
