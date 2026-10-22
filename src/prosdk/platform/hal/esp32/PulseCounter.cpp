// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: PCNT pulse counters (PulseCounter.hpp).

#include "PulseCounter.hpp"

#include <driver/gpio.h>
#include <driver/pulse_cnt.h>
#include <esp_timer.h>

#include <format>
#include <mutex>

#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"

namespace prosdk::hal::esp32 {

namespace {
constexpr int kLowLimit = -32768;
constexpr int kHighLimit = 32767;
constexpr int64_t kSampleUs = 5000;
constexpr int kNumSamples = 64;  // > 300 ms / 5 ms
constexpr int32_t kMinWindowMs = 5;
constexpr int32_t kMaxWindowMs = 300;
constexpr int32_t kDefaultWindowMs = 50;
}  // namespace

struct PulseCounter::State {
  int aGpio = -1;
  int bGpio = -1;
  pcnt_unit_handle_t unit = nullptr;
  pcnt_channel_handle_t chA = nullptr;
  pcnt_channel_handle_t chB = nullptr;
  int32_t windowMs = kDefaultWindowMs;
  // sample ring, written by the sampler under gMutex
  int32_t counts[kNumSamples] = {};
  int64_t times[kNumSamples] = {};
  int head = 0;    // next slot to write
  int filled = 0;  // valid samples
};

namespace {

std::mutex gMutex;
PulseCounter::State* gActive[kNumPulseCounters] = {};
esp_timer_handle_t gSampler = nullptr;

void Sample(void*) {
  std::unique_lock lock(gMutex, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;  // a reader holds it; the next tick samples
  }
  int64_t now = esp_timer_get_time();
  for (PulseCounter::State* s : gActive) {
    if (s == nullptr) {
      continue;
    }
    int value = 0;
    if (pcnt_unit_get_count(s->unit, &value) != ESP_OK) {
      continue;
    }
    s->counts[s->head] = value;
    s->times[s->head] = now;
    s->head = (s->head + 1) % kNumSamples;
    if (s->filled < kNumSamples) {
      ++s->filled;
    }
  }
}

// gMutex held.
void DestroyState(PulseCounter::State* s) {
  if (s->unit != nullptr) {
    pcnt_unit_stop(s->unit);
    pcnt_unit_disable(s->unit);
  }
  if (s->chA != nullptr) {
    pcnt_del_channel(s->chA);
  }
  if (s->chB != nullptr) {
    pcnt_del_channel(s->chB);
  }
  if (s->unit != nullptr) {
    pcnt_unit_remove_watch_point(s->unit, kHighLimit);
    pcnt_unit_remove_watch_point(s->unit, kLowLimit);
    pcnt_del_unit(s->unit);
  }
  for (int gpio : {s->aGpio, s->bGpio}) {
    if (IsValidGpio(gpio)) {
      gpio_reset_pin(static_cast<gpio_num_t>(gpio));
    }
  }
  delete s;
}

void SetSingleEdge(pcnt_channel_handle_t ch, bool risingEdge) {
  pcnt_channel_set_edge_action(
      ch, risingEdge ? PCNT_CHANNEL_EDGE_ACTION_INCREASE : PCNT_CHANNEL_EDGE_ACTION_HOLD,
      risingEdge ? PCNT_CHANNEL_EDGE_ACTION_HOLD : PCNT_CHANNEL_EDGE_ACTION_INCREASE);
}

}  // namespace

int32_t PulseCounter::Create(int aGpio, int bGpio, bool risingEdge,
                             std::unique_ptr<PulseCounter>* out) {
  std::scoped_lock lock(gMutex);
  int slot = -1;
  for (int i = 0; i < kNumPulseCounters; ++i) {
    if (gActive[i] == nullptr) {
      slot = i;
      break;
    }
  }
  if (slot < 0) {
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     "All 4 PCNT units are in use (counters and encoders share them)");
  }
  if (gSampler == nullptr) {
    esp_timer_create_args_t args{};
    args.callback = &Sample;
    args.name = "prosdk-pcnt";
    if (esp_timer_create(&args, &gSampler) != ESP_OK ||
        esp_timer_start_periodic(gSampler, kSampleUs) != ESP_OK) {
      return MakeError(HAL_NO_AVAILABLE_RESOURCES, "PCNT rate timer could not be started");
    }
  }

  auto* s = new State;
  s->aGpio = aGpio;
  s->bGpio = bGpio;
  pcnt_unit_config_t uc{};
  uc.low_limit = kLowLimit;
  uc.high_limit = kHighLimit;
  uc.flags.accum_count = 1;
  esp_err_t err = pcnt_new_unit(&uc, &s->unit);
  if (err != ESP_OK) {
    s->unit = nullptr;
  }
  if (err == ESP_OK) {
    pcnt_glitch_filter_config_t fc{};
    fc.max_glitch_ns = 1000;
    err = pcnt_unit_set_glitch_filter(s->unit, &fc);
  }
  if (err == ESP_OK) {
    pcnt_chan_config_t cc{};
    cc.edge_gpio_num = aGpio;
    cc.level_gpio_num = bGpio >= 0 ? bGpio : -1;
    err = pcnt_new_channel(s->unit, &cc, &s->chA);
    if (err != ESP_OK) {
      s->chA = nullptr;
    }
  }
  if (err == ESP_OK && bGpio >= 0) {
    pcnt_chan_config_t cc{};
    cc.edge_gpio_num = bGpio;
    cc.level_gpio_num = aGpio;
    err = pcnt_new_channel(s->unit, &cc, &s->chB);
    if (err != ESP_OK) {
      s->chB = nullptr;
    }
  }
  if (err == ESP_OK) {
    if (bGpio >= 0) {
      pcnt_channel_set_edge_action(s->chA, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE);
      pcnt_channel_set_level_action(s->chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
      pcnt_channel_set_edge_action(s->chB, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_DECREASE);
      pcnt_channel_set_level_action(s->chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    } else {
      SetSingleEdge(s->chA, risingEdge);
      pcnt_channel_set_level_action(s->chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    }
    gpio_pullup_en(static_cast<gpio_num_t>(aGpio));
    if (bGpio >= 0) {
      gpio_pullup_en(static_cast<gpio_num_t>(bGpio));
    }
    err = pcnt_unit_add_watch_point(s->unit, kHighLimit);
  }
  if (err == ESP_OK) {
    err = pcnt_unit_add_watch_point(s->unit, kLowLimit);
  }
  if (err == ESP_OK) {
    err = pcnt_unit_enable(s->unit);
  }
  if (err == ESP_OK) {
    err = pcnt_unit_clear_count(s->unit);
  }
  if (err == ESP_OK) {
    err = pcnt_unit_start(s->unit);
  }
  if (err != ESP_OK) {
    DestroyState(s);
    return MakeError(err == ESP_ERR_NOT_FOUND ? HAL_NO_AVAILABLE_RESOURCES
                                              : HAL_RESOURCE_IS_ALLOCATED,
                     std::format("PCNT setup failed on GPIO {} ({})", aGpio, EspErrName(err)));
  }
  gActive[slot] = s;
  out->reset(new PulseCounter);
  (*out)->m_state = s;
  return HAL_SUCCESS;
}

PulseCounter::~PulseCounter() {
  if (m_state == nullptr) {
    return;
  }
  std::scoped_lock lock(gMutex);
  for (State*& slot : gActive) {
    if (slot == m_state) {
      slot = nullptr;
    }
  }
  DestroyState(m_state);
  m_state = nullptr;
}

int32_t PulseCounter::GetCount(int32_t* count) {
  int value = 0;
  esp_err_t err = pcnt_unit_get_count(m_state->unit, &value);
  *count = value;
  return err == ESP_OK ? HAL_SUCCESS : HAL_HANDLE_ERROR;
}

int32_t PulseCounter::GetRate(double* countsPerSecond) {
  *countsPerSecond = 0;
  int value = 0;
  if (pcnt_unit_get_count(m_state->unit, &value) != ESP_OK) {
    return HAL_HANDLE_ERROR;
  }
  int64_t now = esp_timer_get_time();
  std::scoped_lock lock(gMutex);
  const State& s = *m_state;
  int back = static_cast<int>(s.windowMs * 1000 / kSampleUs);
  if (back > s.filled) {
    back = s.filled;
  }
  if (back <= 0) {
    return HAL_SUCCESS;
  }
  int index = ((s.head - back) % kNumSamples + kNumSamples) % kNumSamples;
  int64_t dt = now - s.times[index];
  if (dt <= 0) {
    return HAL_SUCCESS;
  }
  *countsPerSecond = static_cast<double>(value - s.counts[index]) * 1e6 / static_cast<double>(dt);
  return HAL_SUCCESS;
}

int32_t PulseCounter::SetRateWindow(int32_t windowMs) {
  if (windowMs < kMinWindowMs || windowMs > kMaxWindowMs) {
    return MakeError(HAL_PARAMETER_OUT_OF_RANGE,
                     std::format("Rate window {} ms out of range. Expect [{}-{}] ms", windowMs,
                                 kMinWindowMs, kMaxWindowMs));
  }
  std::scoped_lock lock(gMutex);
  m_state->windowMs = windowMs;
  return HAL_SUCCESS;
}

int32_t PulseCounter::SetEdge(bool risingEdge) {
  if (m_state->chB != nullptr) {
    return HAL_INCOMPATIBLE_STATE;
  }
  std::scoped_lock lock(gMutex);
  SetSingleEdge(m_state->chA, risingEdge);
  return HAL_SUCCESS;
}

int32_t PulseCounter::Reset() {
  std::scoped_lock lock(gMutex);
  if (pcnt_unit_clear_count(m_state->unit) != ESP_OK) {
    return HAL_HANDLE_ERROR;
  }
  m_state->filled = 0;  // no rate across a reset
  return HAL_SUCCESS;
}

}  // namespace prosdk::hal::esp32
