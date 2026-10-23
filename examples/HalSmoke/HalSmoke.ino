// HalSmoke: exercises the ESP32-S3 HAL backend through the WPILib HAL C API
// (design/hal-esp32.md). Not a robot program: a bring-up check for the board.
//
// Wiring (change the pins below to match your board; none may be a probot-core pin):
//   GPIO 38     DIO output   blinks at 1 Hz (from a HAL Notifier thread)
//   GPIO 39     DIO input    pull-up; printed
//   GPIO 37     PWM output   servo pulse, sweeps 1000..2000 us while enabled, held low if disabled
//   GPIO 41/42  Encoder A/B  quadrature, 4x; count and rate printed
//   GPIO 7      Analog in    0..3.1 V; volts printed
//   GPIO 48     LED strip    8 WS2812 LEDs, a moving dot
//
// Enable from the Driver Station: the PWM output only drives pulses while core's outputs are
// enabled. Output goes to Serial only (115200 baud, opened by probot-core), not to the Driver
// Station console.

#define PROBOT_WIFI_AP_PASSWORD "halsmoke1"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/hal/AddressableLED.h>
#include <prosdk/hal/AnalogInput.h>
#include <prosdk/hal/DIO.h>
#include <prosdk/hal/Encoder.h>
#include <prosdk/hal/HAL.h>
#include <prosdk/hal/Notifier.h>
#include <prosdk/hal/PWM.h>
#include <prosdk/platform/Thread.hpp>
#include <prosdk/util/Synchronization.h>

#include <atomic>
#include <thread>

namespace {

constexpr int kBlinkPin = 38;
constexpr int kInputPin = 39;
constexpr int kPwmPin = 37;
constexpr int kEncoderA = 41;
constexpr int kEncoderB = 42;
constexpr int kAnalogPin = 7;
constexpr int kLedPin = 48;
constexpr int kLedCount = 8;

HAL_DigitalHandle gBlink = HAL_INVALID_HANDLE;
HAL_DigitalHandle gInput = HAL_INVALID_HANDLE;
HAL_DigitalHandle gPwm = HAL_INVALID_HANDLE;
HAL_EncoderHandle gEncoder = HAL_INVALID_HANDLE;
HAL_AnalogInputHandle gAnalog = HAL_INVALID_HANDLE;
HAL_AddressableLEDHandle gLeds = HAL_INVALID_HANDLE;
HAL_NotifierHandle gNotifier = HAL_INVALID_HANDLE;
std::atomic<int> gBlinks{0};
bool gStarted = false;
int gTick = 0;

void Check(const char* what, int32_t status) {
  if (status != 0) {
    Serial.printf("HalSmoke: %s failed: %s\n", what, HAL_GetLastError(&status));
  }
}

// Blinks the DIO output from a Notifier: one alarm every 500 ms.
void BlinkThread() {
  int32_t status = 0;
  HAL_SetNotifierAlarm(gNotifier, 0, 500000000LL, false, false, &status);
  Check("HAL_SetNotifierAlarm", status);
  bool level = false;
  while (WPI_WaitForObject(gNotifier) != 0) {
    level = !level;
    HAL_SetDIO(gBlink, level, &status);
    HAL_AcknowledgeNotifierAlarm(gNotifier, &status);
    ++gBlinks;
  }
}

void Start() {
  if (gStarted) {
    return;
  }
  gStarted = true;
  if (!HAL_Initialize()) {
    Serial.println("HalSmoke: HAL_Initialize failed");
    return;
  }
  int32_t status = 0;
  gBlink = HAL_InitializeDIOPort(kBlinkPin, false, "HalSmoke blink", &status);
  Check("DIO output", status);
  status = 0;
  gInput = HAL_InitializeDIOPort(kInputPin, true, "HalSmoke input", &status);
  Check("DIO input", status);
  status = 0;
  gPwm = HAL_InitializePWMPort(kPwmPin, "HalSmoke PWM", &status);
  Check("PWM", status);
  status = 0;
  gEncoder = HAL_InitializeEncoder(kEncoderA, kEncoderB, false, HAL_ENCODER_4X_ENCODING, &status);
  Check("Encoder", status);
  status = 0;
  gAnalog = HAL_InitializeAnalogInputPort(kAnalogPin, "HalSmoke analog", &status);
  Check("AnalogInput", status);
  status = 0;
  gLeds = HAL_InitializeAddressableLED(kLedPin, "HalSmoke LEDs", &status);
  Check("AddressableLED", status);
  status = 0;
  HAL_SetAddressableLEDLength(gLeds, kLedCount, &status);
  Check("AddressableLED length", status);
  status = 0;
  gNotifier = HAL_CreateNotifier(&status);
  Check("Notifier", status);
  if (status == 0) {
    prosdk::platform::ThreadConfig config;
    config.name = "halsmoke-blink";
    prosdk::platform::ScopedThreadConfig scoped{config};
    std::thread(BlinkThread).detach();
  }
}

void Step() {
  Start();
  int32_t status = 0;
  // PWM sweep (the HAL holds it low while core's outputs are disabled)
  int pulse = 1000 + (gTick * 20) % 1000;
  HAL_SetPWMPulseTimeMicroseconds(gPwm, pulse, &status);
  // moving dot
  HAL_AddressableLEDData leds[kLedCount] = {};
  leds[gTick % kLedCount] = {0, 0, 40};  // r, g, b: blue
  status = 0;
  HAL_SetAddressableLEDData(0, kLedCount, HAL_ALED_GRB, leds, &status);
  if (++gTick % 25 == 0) {  // every 0.5 s at core's 20 ms loop
    int32_t s = 0;
    Serial.printf("HalSmoke: in=%d enc=%d rate=%.1f analog=%.3fV blinks=%d pwm=%dus\n",
                  static_cast<int>(HAL_GetDIO(gInput, &s)), HAL_GetEncoder(gEncoder, &s),
                  HAL_GetEncoderRate(gEncoder, &s), HAL_GetAnalogVoltage(gAnalog, &s),
                  gBlinks.load(), pulse);
  }
}

}  // namespace

void teleopInit() {
  Start();
}
void teleopLoop() {
  Step();
}
void teleopStop() {}
void autonomousLoop() {
  Step();
}
void autonomousStop() {}
