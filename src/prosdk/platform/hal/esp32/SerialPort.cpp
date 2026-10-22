// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: serial ports on the ESP-IDF UART driver (design/hal-esp32.md, "Serial").
// kOnboard is UART1 and kMXP is UART2, on the pins from prosdk::platform::SetSerialPins (onboard
// default RX 15 / TX 16; MXP has no default). UART0 is the console and USB is not a serial port
// here: kUSB1/kUSB2 report HAL_SERIAL_PORT_NOT_FOUND. A UART that Arduino's Serial1/Serial2
// already opened is refused. The RX/TX GPIOs are claimed in the HAL pin table while the port is
// open, so DIO/PWM/... on them fail (and work again after HAL_CloseSerial). Defaults: 9600 8N1, no flow control, read timeout 0 (a read returns
// what is buffered; WPILib's SerialPort sets its own timeout).
// Software flow control and DTR/DSR are not supported; RTS/CTS needs pins and is not offered.

#include "prosdk/hal/SerialPort.h"

#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>

#include <cmath>
#include <format>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>

#include "Esp32Internal.hpp"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/handles/HandlesInternal.hpp"
#include "prosdk/hal/handles/IndexedHandleResource.hpp"
#include "prosdk/platform/HalConfig.hpp"

using namespace prosdk::hal;
using namespace prosdk::hal::esp32;

namespace {

constexpr int kNumPorts = 2;
constexpr int kRxBuffer = 1024;
constexpr int kTxBuffer = 1024;

struct SerialPortState {
  std::string previousAllocation;
  std::mutex mutex;
  uart_port_t uart = UART_NUM_1;
  int rx = -1;
  int tx = -1;
  bool installed = false;
  TickType_t timeoutTicks = 0;
  bool termination = false;
  char terminator = '\n';
  bool flushOnAccess = false;
  HAL_DigitalHandle rxClaim = HAL_INVALID_HANDLE;  // ClaimPin entries of rx/tx
  HAL_DigitalHandle txClaim = HAL_INVALID_HANDLE;

  ~SerialPortState() {
    if (installed) {
      uart_driver_delete(uart);
    }
    for (int gpio : {rx, tx}) {
      if (IsValidGpio(gpio)) {
        gpio_reset_pin(static_cast<gpio_num_t>(gpio));
      }
    }
    ReleasePin(rxClaim, HAL_HandleEnum::SERIAL_PORT);
    ReleasePin(txClaim, HAL_HandleEnum::SERIAL_PORT);
  }
};

IndexedHandleResource<HAL_SerialPortHandle, SerialPortState, kNumPorts,
                      HAL_HandleEnum::SERIAL_PORT>* serialHandles;

std::shared_ptr<SerialPortState> GetPort(HAL_SerialPortHandle handle, int32_t* status) {
  auto port = serialHandles->Get(handle);
  if (port == nullptr) {
    *status = HAL_HANDLE_ERROR;
  } else {
    *status = HAL_SUCCESS;
  }
  return port;
}

int32_t Check(esp_err_t err, const char* what) {
  if (err == ESP_OK) {
    return HAL_SUCCESS;
  }
  return MakeError(HAL_SERIAL_PORT_ERROR,
                   std::format("Serial: {} failed ({})", what, EspErrName(err)));
}

}  // namespace

namespace prosdk::hal::init {
void InitializeSerialPort() {
  static IndexedHandleResource<HAL_SerialPortHandle, SerialPortState, kNumPorts,
                               HAL_HandleEnum::SERIAL_PORT>
      handles;
  serialHandles = &handles;
}
}  // namespace prosdk::hal::init

extern "C" {

HAL_SerialPortHandle HAL_InitializeSerialPort(HAL_SerialPort port, int32_t* status) {
  prosdk::hal::init::CheckInit();
  if (port == HAL_SERIAL_PORT_USB_1 || port == HAL_SERIAL_PORT_USB_2) {
    *status = MakeError(HAL_SERIAL_PORT_NOT_FOUND,
                        "Serial: USB serial ports do not exist on the ESP32-S3 robot (use "
                        "kOnboard or kMXP)");
    return HAL_INVALID_HANDLE;
  }
  if (port != HAL_SERIAL_PORT_ONBOARD && port != HAL_SERIAL_PORT_MXP) {
    *status = MakeErrorIndexOutOfRange(HAL_RESOURCE_OUT_OF_RANGE, "Invalid Index for Serial", 0,
                                       kNumPorts - 1, port);
    return HAL_INVALID_HANDLE;
  }
  const int index = static_cast<int>(port);
  prosdk::platform::SerialConfig config = prosdk::platform::GetSerialConfig(index);
  if (config.rx < 0 && config.tx < 0) {
    *status = MakeError(HAL_SERIAL_PORT_NOT_FOUND,
                        std::format("Serial port {} has no pins: call "
                                    "prosdk::platform::SetSerialPins({}, rx, tx) first",
                                    index, index));
    return HAL_INVALID_HANDLE;
  }
  const uart_port_t uart = index == 0 ? UART_NUM_1 : UART_NUM_2;
  if (uart_is_driver_installed(uart)) {
    *status = MakeError(HAL_SERIAL_PORT_OPEN_ERROR,
                        std::format("Serial: UART{} is already open (Arduino Serial{}?)",
                                    static_cast<int>(uart), static_cast<int>(uart)));
    return HAL_INVALID_HANDLE;
  }
  auto resource = serialHandles->Allocate(static_cast<int16_t>(index), "Serial");
  if (!resource) {
    *status = resource.error();
    return HAL_INVALID_HANDLE;
  }
  auto [handle, state] = *resource;
  state->uart = uart;
  // claim the pins (released by ~SerialPortState, also on the failure paths below)
  const char* portName = index == 0 ? "kOnboard" : "kMXP";
  for (auto [gpio, claim, role] : {std::tuple{config.rx, &state->rxClaim, "RX"},
                                   std::tuple{config.tx, &state->txClaim, "TX"}}) {
    if (gpio < 0) {
      continue;
    }
    auto claimed = ClaimPin(gpio, HAL_HandleEnum::SERIAL_PORT, "Serial",
                            std::format("Serial port {} (UART{} {})", portName,
                                        static_cast<int>(uart), role));
    if (!claimed) {
      serialHandles->Free(handle);
      *status = claimed.error();
      return HAL_INVALID_HANDLE;
    }
    *claim = *claimed;
  }
  state->rx = config.rx;
  state->tx = config.tx;
  uart_config_t uc{};
  uc.baud_rate = 9600;
  uc.data_bits = UART_DATA_8_BITS;
  uc.parity = UART_PARITY_DISABLE;
  uc.stop_bits = UART_STOP_BITS_1;
  uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uc.source_clk = UART_SCLK_DEFAULT;
  esp_err_t err = uart_driver_install(uart, kRxBuffer, kTxBuffer, 0, nullptr, 0);
  if (err == ESP_OK) {
    state->installed = true;
    err = uart_param_config(uart, &uc);
  }
  if (err == ESP_OK) {
    err = uart_set_pin(uart, config.tx, config.rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  }
  if (err != ESP_OK) {
    serialHandles->Free(handle);
    *status = MakeError(HAL_SERIAL_PORT_OPEN_ERROR,
                        std::format("Serial: UART{} could not open ({})", static_cast<int>(uart),
                                    EspErrName(err)));
    return HAL_INVALID_HANDLE;
  }
  *status = HAL_SUCCESS;
  return handle;
}

HAL_SerialPortHandle HAL_InitializeSerialPortDirect(HAL_SerialPort port, const char* portName,
                                                    int32_t* status) {
  (void)portName;  // no device names on the ESP32-S3
  return HAL_InitializeSerialPort(port, status);
}

int HAL_GetSerialFD(HAL_SerialPortHandle handle, int32_t* status) {
  auto port = GetPort(handle, status);
  return -1;  // no file descriptors
}

void HAL_SetSerialBaudRate(HAL_SerialPortHandle handle, int32_t baud, int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return;
  }
  if (baud <= 0) {
    *status = HAL_PARAMETER_OUT_OF_RANGE;
    return;
  }
  std::scoped_lock lock(port->mutex);
  *status = Check(uart_set_baudrate(port->uart, static_cast<uint32_t>(baud)), "baud rate");
}

void HAL_SetSerialDataBits(HAL_SerialPortHandle handle, int32_t bits, int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return;
  }
  uart_word_length_t length;
  switch (bits) {
    case 5:
      length = UART_DATA_5_BITS;
      break;
    case 6:
      length = UART_DATA_6_BITS;
      break;
    case 7:
      length = UART_DATA_7_BITS;
      break;
    case 8:
      length = UART_DATA_8_BITS;
      break;
    default:
      *status = HAL_PARAMETER_OUT_OF_RANGE;
      return;
  }
  std::scoped_lock lock(port->mutex);
  *status = Check(uart_set_word_length(port->uart, length), "data bits");
}

// WPILib SerialPort::Parity: 0 none, 1 odd, 2 even, 3 mark, 4 space.
void HAL_SetSerialParity(HAL_SerialPortHandle handle, int32_t parity, int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return;
  }
  uart_parity_t mode;
  switch (parity) {
    case 0:
      mode = UART_PARITY_DISABLE;
      break;
    case 1:
      mode = UART_PARITY_ODD;
      break;
    case 2:
      mode = UART_PARITY_EVEN;
      break;
    default:
      *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE,
                          "Serial: mark/space parity is not supported on the ESP32-S3");
      return;
  }
  std::scoped_lock lock(port->mutex);
  *status = Check(uart_set_parity(port->uart, mode), "parity");
}

// WPILib SerialPort::StopBits: 10 one, 15 one and a half, 20 two.
void HAL_SetSerialStopBits(HAL_SerialPortHandle handle, int32_t stopBits, int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return;
  }
  uart_stop_bits_t bits;
  switch (stopBits) {
    case 10:
      bits = UART_STOP_BITS_1;
      break;
    case 15:
      bits = UART_STOP_BITS_1_5;
      break;
    case 20:
      bits = UART_STOP_BITS_2;
      break;
    default:
      *status = HAL_PARAMETER_OUT_OF_RANGE;
      return;
  }
  std::scoped_lock lock(port->mutex);
  *status = Check(uart_set_stop_bits(port->uart, bits), "stop bits");
}

// WPILib SerialPort::WriteMode: 1 flush on access, 2 flush when full.
void HAL_SetSerialWriteMode(HAL_SerialPortHandle handle, int32_t mode, int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return;
  }
  std::scoped_lock lock(port->mutex);
  port->flushOnAccess = mode == 1;
}

// WPILib SerialPort::FlowControl: 0 none (the only one supported here).
void HAL_SetSerialFlowControl(HAL_SerialPortHandle handle, int32_t flow, int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return;
  }
  if (flow != 0) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE,
                        "Serial: flow control is not supported by the ESP32-S3 HAL");
  }
}

void HAL_SetSerialTimeout(HAL_SerialPortHandle handle, double timeout, int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return;
  }
  if (!(timeout >= 0) || !std::isfinite(timeout)) {
    *status = HAL_PARAMETER_OUT_OF_RANGE;
    return;
  }
  std::scoped_lock lock(port->mutex);
  port->timeoutTicks = pdMS_TO_TICKS(static_cast<uint32_t>(timeout * 1000.0));
}

void HAL_EnableSerialTermination(HAL_SerialPortHandle handle, char terminator,
                                 int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return;
  }
  std::scoped_lock lock(port->mutex);
  port->termination = true;
  port->terminator = terminator;
}

void HAL_DisableSerialTermination(HAL_SerialPortHandle handle, int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return;
  }
  std::scoped_lock lock(port->mutex);
  port->termination = false;
}

// The driver buffers are fixed (1 KiB each way).
void HAL_SetSerialReadBufferSize(HAL_SerialPortHandle handle, int32_t size, int32_t* status) {
  auto port = GetPort(handle, status);
}

void HAL_SetSerialWriteBufferSize(HAL_SerialPortHandle handle, int32_t size, int32_t* status) {
  auto port = GetPort(handle, status);
}

int32_t HAL_GetSerialBytesReceived(HAL_SerialPortHandle handle, int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return -1;
  }
  size_t size = 0;
  *status = Check(uart_get_buffered_data_len(port->uart, &size), "available");
  return static_cast<int32_t>(size);
}

int32_t HAL_ReadSerial(HAL_SerialPortHandle handle, char* buffer, int32_t count,
                       int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return -1;
  }
  if (count <= 0) {
    return 0;
  }
  std::scoped_lock lock(port->mutex);
  if (!port->termination) {
    int read = uart_read_bytes(port->uart, buffer, static_cast<uint32_t>(count),
                               port->timeoutTicks);
    if (read < 0) {
      *status = MakeError(HAL_SERIAL_PORT_ERROR, "Serial: read failed");
      return -1;
    }
    return read;
  }
  // byte by byte until the terminator, count bytes or the timeout
  TickType_t start = xTaskGetTickCount();
  int32_t read = 0;
  while (read < count) {
    TickType_t elapsed = xTaskGetTickCount() - start;
    TickType_t wait = elapsed >= port->timeoutTicks ? 0 : port->timeoutTicks - elapsed;
    int n = uart_read_bytes(port->uart, buffer + read, 1, wait);
    if (n < 0) {
      *status = MakeError(HAL_SERIAL_PORT_ERROR, "Serial: read failed");
      return -1;
    }
    if (n == 0) {
      break;
    }
    if (buffer[read++] == port->terminator) {
      break;
    }
  }
  return read;
}

int32_t HAL_WriteSerial(HAL_SerialPortHandle handle, const char* buffer, int32_t count,
                        int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return -1;
  }
  if (count <= 0) {
    return 0;
  }
  std::scoped_lock lock(port->mutex);
  int written = uart_write_bytes(port->uart, buffer, static_cast<size_t>(count));
  if (written < 0) {
    *status = MakeError(HAL_SERIAL_PORT_ERROR, "Serial: write failed");
    return -1;
  }
  if (port->flushOnAccess) {
    uart_wait_tx_done(port->uart, pdMS_TO_TICKS(100));
  }
  return written;
}

void HAL_FlushSerial(HAL_SerialPortHandle handle, int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return;
  }
  *status = Check(uart_wait_tx_done(port->uart, pdMS_TO_TICKS(1000)), "flush");
}

void HAL_ClearSerial(HAL_SerialPortHandle handle, int32_t* status) {
  auto port = GetPort(handle, status);
  if (port == nullptr) {
    return;
  }
  std::scoped_lock lock(port->mutex);
  *status = Check(uart_flush_input(port->uart), "clear");
}

void HAL_CloseSerial(HAL_SerialPortHandle handle) {
  serialHandles->Free(handle);
}

}  // extern "C"
