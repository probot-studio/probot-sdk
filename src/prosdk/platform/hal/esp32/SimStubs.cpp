// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// Adapted for prosdk (Probot Studio): the HALSIM_* simulation-data API on the robot, where it has
// no simulator behind it. Concatenated from allwpilib hal/src/main/native/systemcore/mockdata/*.cpp
// (allwpilib b68effc) with the wpi -> prosdk namespace and include renames; the bodies are
// unchanged (stubs: setters do nothing, getters return fixed values, callbacks never fire).
// Not vendored by tools/upstream (hal.py never vendors the systemcore backend); keep in step by
// hand when the HALSIM headers change (the link test, tools/build/hal_link_test.py, catches a
// missing function).

#include "prosdk/hal/DriverStationTypes.h"
#include "prosdk/hal/simulation/AddressableLEDData.h"
#include "prosdk/hal/simulation/AnalogInData.h"
#include "prosdk/hal/simulation/CTREPCMData.h"
#include "prosdk/hal/simulation/CanData.h"
#include "prosdk/hal/simulation/DIOData.h"
#include "prosdk/hal/simulation/DigitalPWMData.h"
#include "prosdk/hal/simulation/DriverStationData.h"
#include "prosdk/hal/simulation/DutyCycleData.h"
#include "prosdk/hal/simulation/EncoderData.h"
#include "prosdk/hal/simulation/I2CData.h"
#include "prosdk/hal/simulation/MockHooks.h"
#include "prosdk/hal/simulation/NotifierData.h"
#include "prosdk/hal/simulation/PWMData.h"
#include "prosdk/hal/simulation/PowerDistributionData.h"
#include "prosdk/hal/simulation/REVPHData.h"
#include "prosdk/hal/simulation/RoboRioData.h"
#include "prosdk/hal/simulation/SimDataValue.hpp"
#include "prosdk/hal/simulation/SimDeviceData.h"

// ---- AddressableLEDData.cpp ----

extern "C" {

void HALSIM_ResetAddressableLEDData(int32_t index) {}

int32_t HALSIM_GetAddressableLEDData(int32_t start, int32_t length,
                                     struct HAL_AddressableLEDData* data) {
  return 0;
}

void HALSIM_SetAddressableLEDData(int32_t start, int32_t length,
                                  const struct HAL_AddressableLEDData* data) {}

#define DEFINE_CAPI(TYPE, CAPINAME, RETURN) \
  HAL_SIMDATAVALUE_STUB_CAPI(TYPE, HALSIM, AddressableLED##CAPINAME, RETURN)

DEFINE_CAPI(HAL_Bool, Initialized, false)
DEFINE_CAPI(int32_t, Start, 0)
DEFINE_CAPI(int32_t, Length, 0)

#undef DEFINE_CAPI
#define DEFINE_CAPI(TYPE, CAPINAME, RETURN)               \
  HAL_SIMCALLBACKREGISTRY_STUB_CAPI_NOINDEX(TYPE, HALSIM, \
                                            AddressableLED##CAPINAME)

DEFINE_CAPI(HAL_ConstBufferCallback, Data, data)

void HALSIM_RegisterAddressableLEDAllCallbacks(int32_t index,
                                               HAL_NotifyCallback callback,
                                               void* param,
                                               HAL_Bool initialNotify) {}
}  // extern "C"

#undef DEFINE_CAPI

// ---- AnalogInData.cpp ----

extern "C" {
void HALSIM_ResetAnalogInData(int32_t index) {}

HAL_SimDeviceHandle HALSIM_GetAnalogInSimDevice(int32_t index) {
  return 0;
}

#define DEFINE_CAPI(TYPE, CAPINAME, RETURN) \
  HAL_SIMDATAVALUE_STUB_CAPI(TYPE, HALSIM, AnalogIn##CAPINAME, RETURN)

DEFINE_CAPI(HAL_Bool, Initialized, false)
DEFINE_CAPI(double, Voltage, 0)

void HALSIM_RegisterAnalogInAllCallbacks(int32_t index,
                                         HAL_NotifyCallback callback,
                                         void* param, HAL_Bool initialNotify) {}
}  // extern "C"

#undef DEFINE_CAPI

// ---- CTREPCMData.cpp ----

extern "C" {
void HALSIM_ResetCTREPCMData(int32_t index) {}

#define DEFINE_CAPI(TYPE, CAPINAME, RETURN) \
  HAL_SIMDATAVALUE_STUB_CAPI(TYPE, HALSIM, CTREPCM##CAPINAME, RETURN)

HAL_SIMDATAVALUE_STUB_CAPI_CHANNEL(HAL_Bool, HALSIM, CTREPCMSolenoidOutput,
                                   false)
DEFINE_CAPI(HAL_Bool, Initialized, false)
DEFINE_CAPI(HAL_Bool, CompressorOn, false)
DEFINE_CAPI(HAL_Bool, ClosedLoopEnabled, false)
DEFINE_CAPI(HAL_Bool, PressureSwitch, false)
DEFINE_CAPI(double, CompressorCurrent, 0)

void HALSIM_GetCTREPCMAllSolenoids(int32_t index, uint8_t* values) {
  *values = 0;
}

void HALSIM_SetCTREPCMAllSolenoids(int32_t index, uint8_t values) {}

void HALSIM_RegisterCTREPCMAllNonSolenoidCallbacks(int32_t index,
                                                   HAL_NotifyCallback callback,
                                                   void* param,
                                                   HAL_Bool initialNotify) {}

void HALSIM_RegisterCTREPCMAllSolenoidCallbacks(int32_t index, int32_t channel,
                                                HAL_NotifyCallback callback,
                                                void* param,
                                                HAL_Bool initialNotify) {}
}  // extern "C"

#undef DEFINE_CAPI

// ---- CanDataInternal.cpp ----

extern "C" {

void HALSIM_ResetCanData(void) {}

#define DEFINE_CAPI(TYPE, CAPINAME) \
  HAL_SIMCALLBACKREGISTRY_STUB_CAPI_NOINDEX(TYPE, HALSIM, Can##CAPINAME)

DEFINE_CAPI(HAL_CAN_SendMessageCallback, SendMessage)
DEFINE_CAPI(HAL_CAN_ReceiveMessageCallback, ReceiveMessage)
DEFINE_CAPI(HAL_CAN_OpenStreamSessionCallback, OpenStream)
DEFINE_CAPI(HAL_CAN_CloseStreamSessionCallback, CloseStream)
DEFINE_CAPI(HAL_CAN_ReadStreamSessionCallback, ReadStream)
DEFINE_CAPI(HAL_CAN_GetCANStatusCallback, GetCANStatus)

}  // extern "C"

#undef DEFINE_CAPI

// ---- DIOData.cpp ----

extern "C" {
void HALSIM_ResetDIOData(int32_t index) {}

HAL_SimDeviceHandle HALSIM_GetDIOSimDevice(int32_t index) {
  return 0;
}

#define DEFINE_CAPI(TYPE, CAPINAME, RETURN) \
  HAL_SIMDATAVALUE_STUB_CAPI(TYPE, HALSIM, DIO##CAPINAME, RETURN)

DEFINE_CAPI(HAL_Bool, Initialized, false)
DEFINE_CAPI(HAL_Bool, Value, false)
DEFINE_CAPI(double, PulseLength, 0)
DEFINE_CAPI(HAL_Bool, IsInput, false)
DEFINE_CAPI(int32_t, FilterIndex, 0)

void HALSIM_RegisterDIOAllCallbacks(int32_t index, HAL_NotifyCallback callback,
                                    void* param, HAL_Bool initialNotify) {}
}  // extern "C"

#undef DEFINE_CAPI

// ---- DigitalPWMData.cpp ----

extern "C" {
int32_t HALSIM_FindDigitalPWMForChannel(int32_t channel) {
  return 0;
}

void HALSIM_ResetDigitalPWMData(int32_t index) {}

#define DEFINE_CAPI(TYPE, CAPINAME, RETURN) \
  HAL_SIMDATAVALUE_STUB_CAPI(TYPE, HALSIM, DigitalPWM##CAPINAME, RETURN)

DEFINE_CAPI(HAL_Bool, Initialized, false)
DEFINE_CAPI(double, DutyCycle, 0)
DEFINE_CAPI(int32_t, Pin, 0)

void HALSIM_RegisterDigitalPWMAllCallbacks(int32_t index,
                                           HAL_NotifyCallback callback,
                                           void* param,
                                           HAL_Bool initialNotify) {}
}  // extern "C"

#undef DEFINE_CAPI

// ---- DriverStationData.cpp ----

extern "C" {
void HALSIM_ResetDriverStationData(void) {}

#define DEFINE_CAPI(TYPE, CAPINAME, RETURN)                                 \
  HAL_SIMDATAVALUE_STUB_CAPI_NOINDEX(TYPE, HALSIM, DriverStation##CAPINAME, \
                                     RETURN)

DEFINE_CAPI(HAL_Bool, Enabled, false)
DEFINE_CAPI(HAL_RobotMode, RobotMode, HAL_ROBOT_MODE_UNKNOWN)
DEFINE_CAPI(HAL_Bool, EStop, false)
DEFINE_CAPI(HAL_Bool, FmsAttached, false)
DEFINE_CAPI(HAL_Bool, DsAttached, false)
DEFINE_CAPI(HAL_AllianceStationID, AllianceStationId,
            HAL_ALLIANCE_STATION_RED_1)
DEFINE_CAPI(double, MatchTime, 0)
DEFINE_CAPI(int64_t, OpMode, 0)

int32_t HALSIM_RegisterOpModeOptionsCallback(HAL_OpModeOptionsCallback callback,
                                             void* param,
                                             HAL_Bool initialNotify) {
  return 0;
}

void HALSIM_CancelOpModeOptionsCallback(int32_t uid) {}

struct HAL_OpModeOption* HALSIM_GetOpModeOptions(int32_t* len) {
  *len = 0;
  return nullptr;
}

void HALSIM_FreeOpModeOptionsArray(struct HAL_OpModeOption* arr,
                                   size_t length) {}

#undef DEFINE_CAPI
#define DEFINE_CAPI(name, data)                                                \
  int32_t HALSIM_RegisterJoystick##name##Callback(                             \
      int32_t joystickNum, HAL_Joystick##name##Callback callback, void* param, \
      HAL_Bool initialNotify) {                                                \
    return 0;                                                                  \
  }                                                                            \
                                                                               \
  void HALSIM_CancelJoystick##name##Callback(int32_t uid) {}                   \
                                                                               \
  void HALSIM_GetJoystick##name(int32_t joystickNum, HAL_Joystick##name* d) {} \
                                                                               \
  void HALSIM_SetJoystick##name(int32_t joystickNum,                           \
                                const HAL_Joystick##name* d) {}

DEFINE_CAPI(Axes, axes)
DEFINE_CAPI(POVs, povs)
DEFINE_CAPI(Buttons, buttons)
DEFINE_CAPI(Touchpads, touchpads)
DEFINE_CAPI(Descriptor, descriptor)

int32_t HALSIM_RegisterJoystickLedsCallback(int32_t joystickNum,
                                            HAL_JoystickLedsCallback callback,
                                            void* param,
                                            HAL_Bool initialNotify) {
  return 0;
}

void HALSIM_CancelJoystickLedsCallback(int32_t uid) {}

void HALSIM_GetJoystickLeds(int32_t joystickNum, int32_t* leds) {}

void HALSIM_SetJoystickLeds(int32_t joystickNum, int32_t leds) {}

int32_t HALSIM_RegisterJoystickRumblesCallback(
    int32_t joystickNum, HAL_JoystickRumblesCallback callback, void* param,
    HAL_Bool initialNotify) {
  return 0;
}

void HALSIM_CancelJoystickRumblesCallback(int32_t uid) {}

void HALSIM_GetJoystickRumbles(int32_t joystickNum, int32_t* leftRumble,
                               int32_t* rightRumble, int32_t* leftTriggerRumble,
                               int32_t* rightTriggerRumble) {}

void HALSIM_SetJoystickRumbles(int32_t joystickNum, int32_t leftRumble,
                               int32_t rightRumble, int32_t leftTriggerRumble,
                               int32_t rightTriggerRumble) {}

int32_t HALSIM_RegisterMatchInfoCallback(HAL_MatchInfoCallback callback,
                                         void* param, HAL_Bool initialNotify) {
  return 0;
}

void HALSIM_CancelMatchInfoCallback(int32_t uid) {}

void HALSIM_GetMatchInfo(HAL_MatchInfo* info) {}

void HALSIM_SetMatchInfo(const HAL_MatchInfo* info) {}

int32_t HALSIM_RegisterGameDataCallback(HAL_GameDataCallback callback,
                                        void* param, HAL_Bool initialNotify) {
  return 0;
}

void HALSIM_CancelGameDataCallback(int32_t uid) {}

void HALSIM_GetGameData(HAL_GameData* info) {}

void HALSIM_SetGameData(const HAL_GameData* info) {}

int32_t HALSIM_RegisterDriverStationNewDataCallback(HAL_NotifyCallback callback,
                                                    void* param,
                                                    HAL_Bool initialNotify) {
  return 0;
}

void HALSIM_CancelDriverStationNewDataCallback(int32_t uid) {}

void HALSIM_NotifyDriverStationNewData(void) {}

void HALSIM_SetJoystickButton(int32_t stick, int32_t button, HAL_Bool state) {}

void HALSIM_SetJoystickAxis(int32_t stick, int32_t axis, double value) {}

void HALSIM_SetJoystickPOV(int32_t stick, int32_t pov, HAL_JoystickPOV value) {}

void HALSIM_SetJoystickButtonsValue(int32_t stick, uint64_t buttons) {}

void HALSIM_SetJoystickAxesAvailable(int32_t stick, uint16_t available) {}

void HALSIM_SetJoystickPOVsAvailable(int32_t stick, uint8_t available) {}

void HALSIM_SetJoystickButtonsAvailable(int32_t stick, uint64_t available) {}

void HALSIM_GetJoystickAvailables(int32_t stick, uint16_t* axesAvailable,
                                  uint64_t* buttonsAvailable,
                                  uint8_t* povsAvailable) {
  *axesAvailable = 0;
  *buttonsAvailable = 0;
  *povsAvailable = 0;
}

void HALSIM_SetJoystickTouchpadCounts(int32_t stick, uint8_t touchpadCount,
                                      const uint8_t* fingerCount) {}

void HALSIM_SetJoystickTouchpadFinger(int32_t stick, int32_t touchpad,
                                      int32_t finger, HAL_Bool down, double x,
                                      double y) {}

void HALSIM_SetJoystickIsGamepad(int32_t stick, HAL_Bool isGamepad) {}

void HALSIM_SetJoystickGamepadType(int32_t stick, int32_t type) {}

void HALSIM_SetJoystickName(int32_t stick, const struct WPI_String* name) {}

void HALSIM_SetJoystickSupportedOutputs(int32_t stick,
                                        int32_t supportedOutputs) {}

void HALSIM_SetGameDataString(const struct WPI_String* message) {}

void HALSIM_SetEventName(const struct WPI_String* name) {}

void HALSIM_SetMatchType(HAL_MatchType type) {}

void HALSIM_SetMatchNumber(int32_t matchNumber) {}

void HALSIM_SetReplayNumber(int32_t replayNumber) {}

void HALSIM_RegisterDriverStationAllCallbacks(HAL_NotifyCallback callback,
                                              void* param,
                                              HAL_Bool initialNotify) {}
}  // extern "C"

#undef DEFINE_CAPI

// ---- DutyCycleData.cpp ----

extern "C" {

void HALSIM_ResetDutyCycleData(int32_t index) {}

HAL_SimDeviceHandle HALSIM_GetDutyCycleSimDevice(int32_t index) {
  return 0;
}

#define DEFINE_CAPI(TYPE, CAPINAME, RETURN) \
  HAL_SIMDATAVALUE_STUB_CAPI(TYPE, HALSIM, DutyCycle##CAPINAME, RETURN)

DEFINE_CAPI(HAL_Bool, Initialized, false)
DEFINE_CAPI(double, Frequency, 0)
DEFINE_CAPI(double, Output, 0)

void HALSIM_RegisterDutyCycleAllCallbacks(int32_t index,
                                          HAL_NotifyCallback callback,
                                          void* param, HAL_Bool initialNotify) {
}

}  // extern "C"

#undef DEFINE_CAPI

// ---- EncoderData.cpp ----

extern "C" {
int32_t HALSIM_FindEncoderForChannel(int32_t channel) {
  return 0;
}

void HALSIM_ResetEncoderData(int32_t index) {}

int32_t HALSIM_GetEncoderDigitalChannelA(int32_t index) {
  return 0;
}

int32_t HALSIM_GetEncoderDigitalChannelB(int32_t index) {
  return 0;
}

HAL_SimDeviceHandle HALSIM_GetEncoderSimDevice(int32_t index) {
  return 0;
}

#define DEFINE_CAPI(TYPE, CAPINAME, RETURN) \
  HAL_SIMDATAVALUE_STUB_CAPI(TYPE, HALSIM, Encoder##CAPINAME, RETURN)

DEFINE_CAPI(HAL_Bool, Initialized, false)
DEFINE_CAPI(int32_t, Count, 0)
DEFINE_CAPI(double, Rate, 0)
DEFINE_CAPI(HAL_Bool, Reset, false)
DEFINE_CAPI(HAL_Bool, Direction, false)
DEFINE_CAPI(HAL_Bool, ReverseDirection, false)
DEFINE_CAPI(double, DistancePerPulse, 0)

void HALSIM_SetEncoderDistance(int32_t index, double distance) {}

double HALSIM_GetEncoderDistance(int32_t index) {
  return 0;
}

void HALSIM_RegisterEncoderAllCallbacks(int32_t index,
                                        HAL_NotifyCallback callback,
                                        void* param, HAL_Bool initialNotify) {}
}  // extern "C"

#undef DEFINE_CAPI

// ---- I2CData.cpp ----

extern "C" {
void HALSIM_ResetI2CData(int32_t index) {}

#define DEFINE_CAPI(TYPE, CAPINAME, RETURN) \
  HAL_SIMDATAVALUE_STUB_CAPI(TYPE, HALSIM, I2C##CAPINAME, RETURN)

DEFINE_CAPI(HAL_Bool, Initialized, false)

#undef DEFINE_CAPI
#define DEFINE_CAPI(TYPE, CAPINAME) \
  HAL_SIMCALLBACKREGISTRY_STUB_CAPI(TYPE, HALSIM, I2C##CAPINAME)

DEFINE_CAPI(HAL_BufferCallback, Read)
DEFINE_CAPI(HAL_ConstBufferCallback, Write)

}  // extern "C"

#undef DEFINE_CAPI

// ---- IMUData.cpp ----

extern "C" {
#define DEFINE_CAPI(TYPE, CAPINAME, RETURN) \
  HAL_SIMDATAVALUE_STUB_CAPI(TYPE, HALSIM, IMU##CAPINAME, RETURN)

DEFINE_CAPI(double, AngleX, 0)
DEFINE_CAPI(double, AngleY, 0)
DEFINE_CAPI(double, AngleZ, 0)

DEFINE_CAPI(double, GyroRateX, 0)
DEFINE_CAPI(double, GyroRateY, 0)
DEFINE_CAPI(double, GyroRateZ, 0)

DEFINE_CAPI(double, AccelX, 0)
DEFINE_CAPI(double, AccelY, 0)
DEFINE_CAPI(double, AccelZ, 0)

DEFINE_CAPI(double, Yaw, 0)

}  // extern "C"

#undef DEFINE_CAPI

// ---- MockHooks.cpp ----

extern "C" {

void HALSIM_SetRuntimeType(HAL_RuntimeType type) {}

void HALSIM_WaitForProgramStart(HAL_Bool waitForFirstNotifier) {}

void HALSIM_SetProgramStarted(HAL_Bool started) {}

HAL_Bool HALSIM_GetProgramStarted(void) {
  return false;
}

void HALSIM_SetProgramState(HAL_ControlWord controlWord) {}

void HALSIM_GetProgramState(HAL_ControlWord* controlWord) {
  controlWord->value = 0;
}

void HALSIM_RestartTiming(void) {}

void HALSIM_PauseTiming(void) {}

void HALSIM_ResumeTiming(void) {}

HAL_Bool HALSIM_IsTimingPaused(void) {
  return false;
}

void HALSIM_StepTiming(int64_t delta) {}

void HALSIM_StepTimingAsync(int64_t delta) {}

void HALSIM_SetSendError(HALSIM_SendErrorHandler handler) {}

void HALSIM_SetSendConsoleLine(HALSIM_SendConsoleLineHandler handler) {}

void HALSIM_SetWriteDisplayAnsi(HALSIM_WriteDisplayAnsiHandler handler) {}

int32_t HALSIM_RegisterSimPeriodicBeforeCallback(
    HALSIM_SimPeriodicCallback callback, void* param) {
  return 0;
}

void HALSIM_CancelSimPeriodicBeforeCallback(int32_t uid) {}

int32_t HALSIM_RegisterSimPeriodicAfterCallback(
    HALSIM_SimPeriodicCallback callback, void* param) {
  return 0;
}

void HALSIM_CancelSimPeriodicAfterCallback(int32_t uid) {}

void HALSIM_CancelAllSimPeriodicCallbacks(void) {}

}  // extern "C"

// ---- NotifierData.cpp ----

extern "C" {

int64_t HALSIM_GetNextNotifierTimeout(void) {
  return 0;
}

int32_t HALSIM_GetNumNotifiers(void) {
  return 0;
}

int32_t HALSIM_GetNotifierInfo(struct HALSIM_NotifierInfo* arr, int32_t size) {
  return 0;
}

}  // extern "C"

// ---- PWMData.cpp ----

extern "C" {
void HALSIM_ResetPWMData(int32_t index) {}

HAL_SimDeviceHandle HALSIM_GetPWMSimDevice(int32_t index) {
  return 0;
}

#define DEFINE_CAPI(TYPE, CAPINAME, RETURN) \
  HAL_SIMDATAVALUE_STUB_CAPI(TYPE, HALSIM, PWM##CAPINAME, RETURN)

DEFINE_CAPI(HAL_Bool, Initialized, false)
DEFINE_CAPI(int32_t, PulseMicrosecond, 0)
DEFINE_CAPI(int32_t, OutputPeriod, 0)

void HALSIM_RegisterPWMAllCallbacks(int32_t index, HAL_NotifyCallback callback,
                                    void* param, HAL_Bool initialNotify) {}
}  // extern "C"

#undef DEFINE_CAPI

// ---- PowerDistributionData.cpp ----

extern "C" {
void HALSIM_ResetPowerDistributionData(int32_t index) {}

#define DEFINE_CAPI(TYPE, CAPINAME, RETURN) \
  HAL_SIMDATAVALUE_STUB_CAPI(TYPE, HALSIM, PowerDistribution##CAPINAME, RETURN)

DEFINE_CAPI(HAL_Bool, Initialized, false)
DEFINE_CAPI(double, Temperature, 0)
DEFINE_CAPI(double, Voltage, 0)
HAL_SIMDATAVALUE_STUB_CAPI_CHANNEL(double, HALSIM, PowerDistributionCurrent, 0)

void HALSIM_GetPowerDistributionAllCurrents(int32_t index, double* currents,
                                            int length) {
  for (int i = 0; i < length; i++) {
    currents[i] = 0;
  }
}

void HALSIM_SetPowerDistributionAllCurrents(int32_t index,
                                            const double* currents,
                                            int length) {}

void HALSIM_RegisterPowerDistributionAllNonCurrentCallbacks(
    int32_t index, int32_t channel, HAL_NotifyCallback callback, void* param,
    HAL_Bool initialNotify) {}
}  // extern "C"

#undef DEFINE_CAPI

// ---- REVPHData.cpp ----

extern "C" {
void HALSIM_ResetREVPHData(int32_t index) {}

#define DEFINE_CAPI(TYPE, CAPINAME, RETURN) \
  HAL_SIMDATAVALUE_STUB_CAPI(TYPE, HALSIM, REVPH##CAPINAME, RETURN)

HAL_SIMDATAVALUE_STUB_CAPI_CHANNEL(HAL_Bool, HALSIM, REVPHSolenoidOutput, false)
DEFINE_CAPI(HAL_Bool, Initialized, false)
DEFINE_CAPI(HAL_Bool, CompressorOn, false)
DEFINE_CAPI(HAL_REVPHCompressorConfigType, CompressorConfigType,
            HAL_REVPH_COMPRESSOR_CONFIG_DISABLED)
DEFINE_CAPI(HAL_Bool, PressureSwitch, false)
DEFINE_CAPI(double, CompressorCurrent, 0)

void HALSIM_GetREVPHAllSolenoids(int32_t index, uint8_t* values) {
  *values = 0;
}

void HALSIM_SetREVPHAllSolenoids(int32_t index, uint8_t values) {}

void HALSIM_RegisterREVPHAllNonSolenoidCallbacks(int32_t index,
                                                 HAL_NotifyCallback callback,
                                                 void* param,
                                                 HAL_Bool initialNotify) {}

void HALSIM_RegisterREVPHAllSolenoidCallbacks(int32_t index, int32_t channel,
                                              HAL_NotifyCallback callback,
                                              void* param,
                                              HAL_Bool initialNotify) {}
}  // extern "C"

#undef DEFINE_CAPI

// ---- Reset.cpp ----

extern "C" void HALSIM_ResetAllSimData(void) {}

// ---- RoboRioData.cpp ----

extern "C" {
void HALSIM_ResetRoboRioData(void) {}

#define DEFINE_CAPI(TYPE, CAPINAME, RETURN) \
  HAL_SIMDATAVALUE_STUB_CAPI_NOINDEX(TYPE, HALSIM, RoboRio##CAPINAME, RETURN)

DEFINE_CAPI(double, VInVoltage, 0)
DEFINE_CAPI(double, UserVoltage3V3, 0)
DEFINE_CAPI(double, UserCurrent3V3, 0)
DEFINE_CAPI(HAL_Bool, UserActive3V3, false)
DEFINE_CAPI(int32_t, UserFaults3V3, 0)
DEFINE_CAPI(double, BrownoutVoltage, 6.75)
DEFINE_CAPI(double, BrownoutRecoveryVoltage, 7.25)
DEFINE_CAPI(double, CPUTemp, 45.0)
DEFINE_CAPI(int32_t, TeamNumber, 0)

int32_t HALSIM_RegisterRoboRioSerialNumberCallback(
    HAL_RoboRioStringCallback callback, void* param, HAL_Bool initialNotify) {
  return 0;
}
void HALSIM_CancelRoboRioSerialNumberCallback(int32_t uid) {}
void HALSIM_GetRoboRioSerialNumber(struct WPI_String* serialNumber) {
  WPI_AllocateString(serialNumber, 0);
}
void HALSIM_SetRoboRioSerialNumber(const struct WPI_String* serialNumber) {}

int32_t HALSIM_RegisterRoboRioCommentsCallback(
    HAL_RoboRioStringCallback callback, void* param, HAL_Bool initialNotify) {
  return 0;
}
void HALSIM_CancelRoboRioCommentsCallback(int32_t uid) {}
void HALSIM_GetRoboRioComments(struct WPI_String* comments) {
  WPI_AllocateString(comments, 0);
}
void HALSIM_SetRoboRioComments(const struct WPI_String* comments) {}

void HALSIM_RegisterRoboRioAllCallbacks(HAL_NotifyCallback callback,
                                        void* param, HAL_Bool initialNotify) {}
}  // extern "C"

#undef DEFINE_CAPI

// ---- SimDeviceData.cpp ----

extern "C" {

void HALSIM_SetSimDeviceEnabled(const char* prefix, HAL_Bool enabled) {}

HAL_Bool HALSIM_IsSimDeviceEnabled(const char* name) {
  return false;
}

int32_t HALSIM_RegisterSimDeviceCreatedCallback(
    const char* prefix, void* param, HALSIM_SimDeviceCallback callback,
    HAL_Bool initialNotify) {
  return 0;
}

void HALSIM_CancelSimDeviceCreatedCallback(int32_t uid) {}

int32_t HALSIM_RegisterSimDeviceFreedCallback(const char* prefix, void* param,
                                              HALSIM_SimDeviceCallback callback,
                                              HAL_Bool initialNotify) {
  return 0;
}

void HALSIM_CancelSimDeviceFreedCallback(int32_t uid) {}

HAL_SimDeviceHandle HALSIM_GetSimDeviceHandle(const char* name) {
  return 0;
}

const char* HALSIM_GetSimDeviceName(HAL_SimDeviceHandle handle) {
  return "";
}

HAL_SimDeviceHandle HALSIM_GetSimValueDeviceHandle(HAL_SimValueHandle handle) {
  return 0;
}

void HALSIM_EnumerateSimDevices(const char* prefix, void* param,
                                HALSIM_SimDeviceCallback callback) {}

int32_t HALSIM_RegisterSimValueCreatedCallback(HAL_SimDeviceHandle device,
                                               void* param,
                                               HALSIM_SimValueCallback callback,
                                               HAL_Bool initialNotify) {
  return 0;
}

void HALSIM_CancelSimValueCreatedCallback(int32_t uid) {}

int32_t HALSIM_RegisterSimValueChangedCallback(HAL_SimValueHandle handle,
                                               void* param,
                                               HALSIM_SimValueCallback callback,
                                               HAL_Bool initialNotify) {
  return 0;
}

void HALSIM_CancelSimValueChangedCallback(int32_t uid) {}

int32_t HALSIM_RegisterSimValueResetCallback(HAL_SimValueHandle handle,
                                             void* param,
                                             HALSIM_SimValueCallback callback,
                                             HAL_Bool initialNotify) {
  return 0;
}

void HALSIM_CancelSimValueResetCallback(int32_t uid) {}

HAL_SimValueHandle HALSIM_GetSimValueHandle(HAL_SimDeviceHandle device,
                                            const char* name) {
  return 0;
}

void HALSIM_EnumerateSimValues(HAL_SimDeviceHandle device, void* param,
                               HALSIM_SimValueCallback callback) {}

const char** HALSIM_GetSimValueEnumOptions(HAL_SimValueHandle handle,
                                           int32_t* numOptions) {
  *numOptions = 0;
  return nullptr;
}

const double* HALSIM_GetSimValueEnumDoubleValues(HAL_SimValueHandle handle,
                                                 int32_t* numOptions) {
  *numOptions = 0;
  return nullptr;
}

void HALSIM_ResetSimDeviceData(void) {}

}  // extern "C"
