// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: pulse-train I/O on the RMT peripheral (prosdk/platform/hal/PulseIO.hpp,
// design/boardoza/F3-timed-io.md). Both directions run at 1 MHz (1 us per tick, the 80 MHz APB
// clock divided by 80).
//
// TX: one RMT TX channel per transmitter, taken from the 3 that AddressableLED outputs share
// (rmt::AcquireTxChannel; probot-core's status LED has the 4th). A train is a list of RMT symbols
// sent through the copy encoder, optionally modulated on the RMT carrier (high levels only). The
// pin is a HAL output pin (SetOutputPin): the emergency stop routes it to plain GPIO and drives it
// low, and after it no transmitter opens and no train starts. Not gated while disabled (like LED
// strips).
//
// RX: one RMT RX channel per capture (4 on the S3), always armed. The receive-done callback runs
// in the RMT interrupt: it copies the symbols into a lock-free single-producer ring of packed
// pulses and re-arms the receiver after the last part of a burst (partial receive, so a burst
// longer than the receive buffer arrives in pieces). The callback's context is a static slot that
// is never freed, so a late interrupt never touches freed memory.
//
// Unverified on hardware (design/boardoza/F3-timed-io.md): carrier frequency and duty accuracy,
// the glitch filter clock, the partial-receive re-arm under bursts longer than the buffer, the
// RX pull-up after rmt_new_rx_channel, and that the RMT driver does not re-route a TX pin that
// the emergency stop moved to plain GPIO.

#include <driver/gpio.h>
#include <driver/rmt_encoder.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <esp_rom_gpio.h>
#include <freertos/FreeRTOS.h>
#include <soc/gpio_sig_map.h>

#include <atomic>
#include <format>
#include <memory>
#include <mutex>
#include <vector>

#include "Esp32Internal.hpp"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/handles/HandlesInternal.hpp"
#include "prosdk/platform/hal/PulseIO.hpp"

using namespace prosdk::hal;
using namespace prosdk::hal::esp32;
using prosdk::platform::hal::kInvalidPulseHandle;
using prosdk::platform::hal::Pulse;

namespace {

constexpr uint32_t kResolutionHz = 1000000;          // 1 us per tick
constexpr size_t kMemBlockSymbols = 48;              // SOC_RMT_MEM_WORDS_PER_CHANNEL on the S3
constexpr size_t kRxSymbols = 2 * kMemBlockSymbols;  // receive buffer (partial receive)
constexpr int kTxTimeoutMs = 100;
constexpr uint32_t kRxRingSize = prosdk::platform::hal::kRxBufferPulses;
static_assert((kRxRingSize & (kRxRingSize - 1)) == 0, "the ring index wraps at 2^32");

constexpr const char* kEStoppedTx =
    "pulse TX: the robot is emergency stopped; outputs stay off until reboot";

// Routes a pin back to the plain GPIO output, driven low (same as the emergency stop).
void ForceLow(int gpio) {
  gpio_set_level(static_cast<gpio_num_t>(gpio), 0);
  esp_rom_gpio_connect_out_signal(gpio, SIG_GPIO_OUT_IDX, false, false);
}

// ---- TX ----

struct PulseTx final : PinDriver {
  int gpio = -1;
  rmt_channel_handle_t channel = nullptr;
  rmt_encoder_handle_t encoder = nullptr;
  bool rmtReserved = false;          // holds one of the shared RMT TX channels
  std::mutex mutex;                  // one train at a time
  std::vector<rmt_symbol_word_t> symbols;  // the train in flight (read by the RMT driver)
  int64_t trainUs = 0;               // length of the train in flight

  ~PulseTx() override {
    if (channel != nullptr) {
      rmt_tx_wait_all_done(channel, kTxTimeoutMs);
      rmt_disable(channel);
      rmt_del_channel(channel);
    }
    if (encoder != nullptr) {
      rmt_del_encoder(encoder);
    }
    if (rmtReserved) {
      rmt::ReleaseTxChannel();
    }
    if (IsValidGpio(gpio)) {
      SetOutputPin(gpio, false);
      if (EStopped()) {
        // keep the line low (deleting the RMT channel may release the pad)
        gpio_set_direction(static_cast<gpio_num_t>(gpio), GPIO_MODE_OUTPUT);
        ForceLow(gpio);
      } else {
        gpio_reset_pin(static_cast<gpio_num_t>(gpio));
      }
    }
  }
};

HAL_Status StartTx(PulseTx& tx, uint32_t carrierHz, double carrierDuty) {
  if (!rmt::AcquireTxChannel()) {
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     std::format("pulse TX: all {} RMT TX channels are in use (LED outputs and "
                                 "pulse transmitters share them)",
                                 kNumRmtTxChannels));
  }
  tx.rmtReserved = true;
  rmt_tx_channel_config_t cc{};
  cc.gpio_num = static_cast<gpio_num_t>(tx.gpio);
  cc.clk_src = RMT_CLK_SRC_DEFAULT;
  cc.resolution_hz = kResolutionHz;
  cc.mem_block_symbols = kMemBlockSymbols;
  cc.trans_queue_depth = 1;
  esp_err_t err = rmt_new_tx_channel(&cc, &tx.channel);
  if (err != ESP_OK) {
    tx.channel = nullptr;
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     std::format("pulse TX: no RMT TX channel left for GPIO {} ({})", tx.gpio,
                                 EspErrName(err)));
  }
  if (carrierHz != 0) {
    // high levels become carrier bursts (polarity active high), silence between transmissions
    rmt_carrier_config_t carrier{};
    carrier.frequency_hz = carrierHz;
    carrier.duty_cycle = static_cast<float>(carrierDuty);
    err = rmt_apply_carrier(tx.channel, &carrier);
  }
  if (err == ESP_OK) {
    rmt_copy_encoder_config_t ec{};
    err = rmt_new_copy_encoder(&ec, &tx.encoder);
    if (err != ESP_OK) {
      tx.encoder = nullptr;
    }
  }
  if (err == ESP_OK) {
    err = rmt_enable(tx.channel);
  }
  if (err != ESP_OK) {
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     std::format("pulse TX: RMT setup failed on GPIO {} ({})", tx.gpio,
                                 EspErrName(err)));
  }
  return HAL_SUCCESS;
}

PulseTx* GetTx(int32_t handle, std::shared_ptr<SmartIo>* port, int32_t* status) {
  *port = smartIoHandles == nullptr ? nullptr : smartIoHandles->Get(handle, HAL_HandleEnum::VENDOR);
  if (*port == nullptr || (*port)->mode != PinMode::kPulseTx) {
    *status = HAL_HANDLE_ERROR;
    return nullptr;
  }
  return static_cast<PulseTx*>((*port)->driver.get());
}

// Longest wait for the train in flight: its whole length plus kTxTimeoutMs.
int WaitBudgetMs(const PulseTx& tx) {
  return static_cast<int>(tx.trainUs / 1000) + kTxTimeoutMs;
}

// ---- RX ----

// One RMT RX channel. Static storage: the interrupt callback's context stays valid forever.
struct RxSlot {
  bool inUse = false;  // gRxMutex
  rmt_channel_handle_t channel = nullptr;
  rmt_receive_config_t config{};
  std::atomic<bool> armed{false};  // the callback re-arms the receiver while set
  portMUX_TYPE armLock = portMUX_INITIALIZER_UNLOCKED;  // armed check + re-arm vs. close
  rmt_symbol_word_t symbols[kRxSymbols];
  uint32_t ring[kRxRingSize];      // packed pulses: bit 31 = level, bits 0-30 = us
  std::atomic<uint32_t> head{0};   // written by the interrupt
  std::atomic<uint32_t> tail{0};   // written by ReadPulses
  std::atomic<uint32_t> dropped{0};
};

std::mutex gRxMutex;
RxSlot gRx[kNumPulseRx];

void Push(RxSlot& s, uint32_t level, uint32_t us) {
  uint32_t head = s.head.load(std::memory_order_relaxed);
  if (head - s.tail.load(std::memory_order_acquire) >= kRxRingSize) {
    s.dropped.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  s.ring[head % kRxRingSize] = (level != 0 ? 0x80000000u : 0u) | us;
  s.head.store(head + 1, std::memory_order_release);
}

// RMT interrupt. A symbol half with duration 0 is the level that outlasted the idle threshold:
// the end of the burst.
bool OnRxDone(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t* data, void* context) {
  auto* s = static_cast<RxSlot*>(context);
  for (size_t i = 0; i < data->num_symbols; ++i) {
    const rmt_symbol_word_t& sym = data->received_symbols[i];
    Push(*s, sym.level0, sym.duration0);
    if (sym.duration0 == 0) {
      break;
    }
    Push(*s, sym.level1, sym.duration1);
    if (sym.duration1 == 0) {
      break;
    }
  }
  if (data->flags.is_last) {
    // under armLock: once PulseRx's destructor cleared `armed`, no re-arm can follow (the other
    // core may run this interrupt while the channel is being disabled and deleted)
    portENTER_CRITICAL_ISR(&s->armLock);
    if (s->armed.load(std::memory_order_acquire)) {
      rmt_receive(channel, s->symbols, sizeof(s->symbols), &s->config);
    }
    portEXIT_CRITICAL_ISR(&s->armLock);
  }
  return false;
}

struct PulseRx final : PinDriver {
  int gpio = -1;
  RxSlot* slot = nullptr;
  std::mutex readMutex;  // one reader at a time (the ring has one consumer)

  ~PulseRx() override {
    if (slot != nullptr) {
      portENTER_CRITICAL(&slot->armLock);
      slot->armed.store(false, std::memory_order_release);
      portEXIT_CRITICAL(&slot->armLock);
      if (slot->channel != nullptr) {
        rmt_disable(slot->channel);
        rmt_del_channel(slot->channel);
        slot->channel = nullptr;
      }
      std::scoped_lock lock(gRxMutex);
      slot->inUse = false;
    }
    if (IsValidGpio(gpio)) {
      gpio_reset_pin(static_cast<gpio_num_t>(gpio));
    }
  }
};

HAL_Status StartRx(PulseRx& rx, uint32_t glitchUs, uint32_t idleUs) {
  RxSlot& s = *rx.slot;
  s.head.store(0, std::memory_order_relaxed);
  s.tail.store(0, std::memory_order_relaxed);
  s.dropped.store(0, std::memory_order_relaxed);
  // input with the internal pull-up that gpio_reset_pin enables, as a DIO input (the RMT driver
  // connects the input signal; that it keeps the pull-up is unverified)
  gpio_reset_pin(static_cast<gpio_num_t>(rx.gpio));
  rmt_rx_channel_config_t cc{};
  cc.gpio_num = static_cast<gpio_num_t>(rx.gpio);
  cc.clk_src = RMT_CLK_SRC_DEFAULT;
  cc.resolution_hz = kResolutionHz;
  cc.mem_block_symbols = kMemBlockSymbols;
  esp_err_t err = rmt_new_rx_channel(&cc, &s.channel);
  if (err != ESP_OK) {
    s.channel = nullptr;
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     std::format("pulse RX: no RMT RX channel left for GPIO {} ({})", rx.gpio,
                                 EspErrName(err)));
  }
  rmt_rx_event_callbacks_t cbs{};
  cbs.on_recv_done = &OnRxDone;
  err = rmt_rx_register_event_callbacks(s.channel, &cbs, &s);
  if (err == ESP_OK) {
    err = rmt_enable(s.channel);
  }
  if (err == ESP_OK) {
    s.config = rmt_receive_config_t{};
    // RMT filter: at most 255 APB cycles (RMT_LL_MAX_FILTER_VALUE; filter clock unverified)
    s.config.signal_range_min_ns = glitchUs * 1000;
    s.config.signal_range_max_ns = idleUs * 1000;    // idle threshold: at most 32767 ticks
    s.config.flags.en_partial_rx = 1;
    s.armed.store(true, std::memory_order_release);
    err = rmt_receive(s.channel, s.symbols, sizeof(s.symbols), &s.config);
  }
  if (err != ESP_OK) {
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     std::format("pulse RX: RMT setup failed on GPIO {} ({})", rx.gpio,
                                 EspErrName(err)));
  }
  return HAL_SUCCESS;
}

PulseRx* GetRx(int32_t handle, std::shared_ptr<SmartIo>* port, int32_t* status) {
  *port = smartIoHandles == nullptr ? nullptr : smartIoHandles->Get(handle, HAL_HandleEnum::VENDOR);
  if (*port == nullptr || (*port)->mode != PinMode::kPulseRx) {
    *status = HAL_HANDLE_ERROR;
    return nullptr;
  }
  return static_cast<PulseRx*>((*port)->driver.get());
}

}  // namespace

namespace prosdk::platform::hal {

int32_t OpenPulseTx(int32_t gpio, uint32_t carrierHz, double carrierDuty,
                    const char* allocationLocation, int32_t* status) {
  prosdk::hal::init::CheckInit();
  if (const char* error = detail::CheckPulseTx(carrierHz, carrierDuty)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE, error);
    return kInvalidPulseHandle;
  }
  if (EStopped()) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE, kEStoppedTx);
    return kInvalidPulseHandle;
  }
  auto resource = AllocateSmartIo(gpio, HAL_HandleEnum::VENDOR, "Pulse TX");
  if (!resource) {
    *status = resource.error();
    return kInvalidPulseHandle;
  }
  auto [handle, port] = *resource;
  auto tx = std::make_unique<PulseTx>();
  tx->gpio = gpio;
  *status = StartTx(*tx, carrierHz, carrierDuty);
  if (*status == HAL_SUCCESS) {
    SetOutputPin(gpio, true);  // the emergency stop drives the line low
    if (EStopped()) {
      // the emergency stop ran before the pin was marked: drive it low here and refuse
      ForceLow(gpio);
      *status = MakeError(HAL_INCOMPATIBLE_STATE, kEStoppedTx);
    }
  }
  if (*status != HAL_SUCCESS) {
    tx.reset();
    smartIoHandles->Free(handle, HAL_HandleEnum::VENDOR);
    return kInvalidPulseHandle;
  }
  port->mode = PinMode::kPulseTx;
  port->driver = std::move(tx);
  port->previousAllocation = allocationLocation ? allocationLocation : "";
  return handle;
}

void SendPulses(int32_t handle, std::span<const Pulse> pulses, int32_t* status) {
  std::shared_ptr<SmartIo> port;
  PulseTx* tx = GetTx(handle, &port, status);
  if (tx == nullptr) {
    return;
  }
  if (const char* error = detail::CheckPulseTrain(pulses)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE, error);
    return;
  }
  if (EStopped()) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE, kEStoppedTx);
    return;
  }
  std::scoped_lock lock(tx->mutex);
  if (pulses.empty()) {
    *status = HAL_SUCCESS;
    return;
  }
  // the buffer of the previous train is in use until it is sent
  if (rmt_tx_wait_all_done(tx->channel, WaitBudgetMs(*tx)) != ESP_OK) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE,
                        std::format("pulse TX: GPIO {}: the previous train did not finish",
                                    tx->gpio));
    return;
  }
  // two pulses per RMT symbol; an odd train ends with 1 us low (the idle level follows anyway)
  size_t count = (pulses.size() + 1) / 2;
  tx->symbols.resize(count);
  int64_t totalUs = 0;
  for (size_t i = 0; i < count; ++i) {
    const Pulse& a = pulses[2 * i];
    Pulse b = 2 * i + 1 < pulses.size() ? pulses[2 * i + 1] : Pulse{false, 1};
    rmt_symbol_word_t& sym = tx->symbols[i];
    sym.val = 0;
    sym.level0 = a.level ? 1 : 0;
    sym.duration0 = static_cast<uint16_t>(a.us);
    sym.level1 = b.level ? 1 : 0;
    sym.duration1 = static_cast<uint16_t>(b.us);
    totalUs += a.us + b.us;
  }
  rmt_transmit_config_t tc{};
  tc.loop_count = 0;
  tc.flags.eot_level = 0;
  esp_err_t err = rmt_transmit(tx->channel, tx->encoder, tx->symbols.data(),
                               count * sizeof(rmt_symbol_word_t), &tc);
  if (err != ESP_OK) {
    tx->trainUs = 0;
    *status = MakeError(HAL_INCOMPATIBLE_STATE,
                        std::format("pulse TX: GPIO {} transmit failed ({})", tx->gpio,
                                    EspErrName(err)));
    return;
  }
  tx->trainUs = totalUs;
  // the emergency stop may have run since the check above: keep the line low
  if (EStopped()) {
    ForceLow(tx->gpio);
    *status = MakeError(HAL_INCOMPATIBLE_STATE, kEStoppedTx);
    return;
  }
  *status = HAL_SUCCESS;
}

bool WaitPulsesSent(int32_t handle, int32_t timeoutMs, int32_t* status) {
  std::shared_ptr<SmartIo> port;
  PulseTx* tx = GetTx(handle, &port, status);
  if (tx == nullptr) {
    return false;
  }
  std::scoped_lock lock(tx->mutex);
  *status = HAL_SUCCESS;
  return rmt_tx_wait_all_done(tx->channel, timeoutMs > 0 ? timeoutMs : 0) == ESP_OK;
}

int32_t OpenPulseRx(int32_t gpio, uint32_t glitchUs, uint32_t idleUs,
                    const char* allocationLocation, int32_t* status) {
  prosdk::hal::init::CheckInit();
  if (const char* error = detail::CheckPulseRx(glitchUs, idleUs)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE, error);
    return kInvalidPulseHandle;
  }
  auto resource = AllocateSmartIo(gpio, HAL_HandleEnum::VENDOR, "Pulse RX");
  if (!resource) {
    *status = resource.error();
    return kInvalidPulseHandle;
  }
  auto [handle, port] = *resource;
  auto rx = std::make_unique<PulseRx>();
  rx->gpio = gpio;
  {
    std::scoped_lock lock(gRxMutex);
    for (RxSlot& s : gRx) {
      if (!s.inUse) {
        s.inUse = true;
        rx->slot = &s;
        break;
      }
    }
  }
  *status = rx->slot == nullptr
                ? MakeError(HAL_NO_AVAILABLE_RESOURCES,
                            std::format("pulse RX: all {} RMT RX channels are in use", kNumPulseRx))
                : StartRx(*rx, glitchUs, idleUs);
  if (*status != HAL_SUCCESS) {
    rx.reset();
    smartIoHandles->Free(handle, HAL_HandleEnum::VENDOR);
    return kInvalidPulseHandle;
  }
  port->mode = PinMode::kPulseRx;
  port->driver = std::move(rx);
  port->previousAllocation = allocationLocation ? allocationLocation : "";
  return handle;
}

int32_t ReadPulses(int32_t handle, std::span<Pulse> out, int32_t* status) {
  std::shared_ptr<SmartIo> port;
  PulseRx* rx = GetRx(handle, &port, status);
  if (rx == nullptr) {
    return 0;
  }
  std::scoped_lock lock(rx->readMutex);
  RxSlot& s = *rx->slot;
  uint32_t tail = s.tail.load(std::memory_order_relaxed);
  uint32_t head = s.head.load(std::memory_order_acquire);
  size_t n = 0;
  while (n < out.size() && tail != head) {
    uint32_t packed = s.ring[tail % kRxRingSize];
    out[n++] = Pulse{(packed & 0x80000000u) != 0, packed & 0x7FFFFFFFu};
    ++tail;
  }
  s.tail.store(tail, std::memory_order_release);
  *status = HAL_SUCCESS;
  return static_cast<int32_t>(n);
}

int64_t GetDroppedPulses(int32_t handle, int32_t* status) {
  std::shared_ptr<SmartIo> port;
  PulseRx* rx = GetRx(handle, &port, status);
  if (rx == nullptr) {
    return 0;
  }
  *status = HAL_SUCCESS;
  return rx->slot->dropped.load(std::memory_order_relaxed);
}

void ClosePulse(int32_t handle) {
  if (smartIoHandles == nullptr) {
    return;
  }
  auto port = smartIoHandles->Get(handle, HAL_HandleEnum::VENDOR);
  // VENDOR handles are also ClaimPin claims (SPI.cpp): close only pulse channels
  if (port == nullptr || (port->mode != PinMode::kPulseTx && port->mode != PinMode::kPulseRx)) {
    return;
  }
  const char* what = port->mode == PinMode::kPulseTx ? "Pulse TX" : "Pulse RX";
  FreeSmartIo(handle, HAL_HandleEnum::VENDOR, std::move(port), what);
}

}  // namespace prosdk::platform::hal
