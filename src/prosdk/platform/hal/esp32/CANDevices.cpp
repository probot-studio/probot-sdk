// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: FRC CAN power and pneumatics devices (CTRE PCM, REV PH, CTRE PDP, REV PDH;
// design/hal-esp32.md, "Not supported"). The Probot kit has none of them and they need the FRC
// CAN heartbeat, which this backend does not send. Initialize* fails with a message; every other
// call reports HAL_HANDLE_ERROR (the handle can never be valid) and returns 0/false. Generated
// once from design/hal-esp32-functions.txt; edit by hand.

#include <cstring>
#include <string>

#include "Esp32Internal.hpp"
#include "prosdk/hal/CTREPCM.h"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/PowerDistribution.h"
#include "prosdk/hal/REVPH.h"

using namespace prosdk::hal;

namespace {
int32_t NotSupported(const char* device) {
  return MakeError(HAL_HANDLE_ERROR,
                   std::string(device) + " is not supported by the ESP32-S3 HAL (no FRC CAN "
                   "devices)");
}
}  // namespace

extern "C" {

// ---- prosdk/hal/CTREPCM.h ----

HAL_Bool HAL_CheckCTREPCMSolenoidChannel(int32_t channel) {
  return 0;
}

void HAL_ClearAllCTREPCMStickyFaults(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_FireCTREPCMOneShot(HAL_CTREPCMHandle handle, int32_t index, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_FreeCTREPCM(HAL_CTREPCMHandle handle) {}

HAL_Bool HAL_GetCTREPCMClosedLoopControl(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_Bool HAL_GetCTREPCMCompressor(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

double HAL_GetCTREPCMCompressorCurrent(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_Bool HAL_GetCTREPCMCompressorCurrentTooHighFault(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_Bool HAL_GetCTREPCMCompressorCurrentTooHighStickyFault(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_Bool HAL_GetCTREPCMCompressorNotConnectedFault(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_Bool HAL_GetCTREPCMCompressorNotConnectedStickyFault(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_Bool HAL_GetCTREPCMCompressorShortedFault(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_Bool HAL_GetCTREPCMCompressorShortedStickyFault(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_Bool HAL_GetCTREPCMPressureSwitch(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

int32_t HAL_GetCTREPCMSolenoidDisabledList(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_Bool HAL_GetCTREPCMSolenoidVoltageFault(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_Bool HAL_GetCTREPCMSolenoidVoltageStickyFault(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

int32_t HAL_GetCTREPCMSolenoids(HAL_CTREPCMHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_CTREPCMHandle HAL_InitializeCTREPCM(int32_t busId, int32_t module, const char* allocationLocation, int32_t* status) {
  *status = NotSupported("CTRE PCM");
  return HAL_INVALID_HANDLE;
}

void HAL_SetCTREPCMClosedLoopControl(HAL_CTREPCMHandle handle, HAL_Bool enabled, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_SetCTREPCMOneShotDuration(HAL_CTREPCMHandle handle, int32_t index, int32_t durMs, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_SetCTREPCMSolenoids(HAL_CTREPCMHandle handle, int32_t mask, int32_t values, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

// ---- prosdk/hal/REVPH.h ----

HAL_Bool HAL_CheckREVPHModuleNumber(int32_t module) {
  return 0;
}

HAL_Bool HAL_CheckREVPHSolenoidChannel(int32_t channel) {
  return 0;
}

void HAL_ClearREVPHStickyFaults(HAL_REVPHHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_FireREVPHOneShot(HAL_REVPHHandle handle, int32_t index, int32_t durMs, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_FreeREVPH(HAL_REVPHHandle handle) {}

double HAL_GetREVPH5VVoltage(HAL_REVPHHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

double HAL_GetREVPHAnalogVoltage(HAL_REVPHHandle handle, int32_t channel, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_Bool HAL_GetREVPHCompressor(HAL_REVPHHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_REVPHCompressorConfigType HAL_GetREVPHCompressorConfig(HAL_REVPHHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return static_cast<HAL_REVPHCompressorConfigType>(0);
}

double HAL_GetREVPHCompressorCurrent(HAL_REVPHHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

void HAL_GetREVPHFaults(HAL_REVPHHandle handle, HAL_REVPHFaults* faults, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  if (faults != nullptr) {
    std::memset(faults, 0, sizeof(*faults));
  }
}

HAL_Bool HAL_GetREVPHPressureSwitch(HAL_REVPHHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

double HAL_GetREVPHSolenoidCurrent(HAL_REVPHHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

int32_t HAL_GetREVPHSolenoidDisabledList(HAL_REVPHHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

double HAL_GetREVPHSolenoidVoltage(HAL_REVPHHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

int32_t HAL_GetREVPHSolenoids(HAL_REVPHHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

void HAL_GetREVPHStickyFaults(HAL_REVPHHandle handle, HAL_REVPHStickyFaults* stickyFaults, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  if (stickyFaults != nullptr) {
    std::memset(stickyFaults, 0, sizeof(*stickyFaults));
  }
}

void HAL_GetREVPHVersion(HAL_REVPHHandle handle, HAL_REVPHVersion* version, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  if (version != nullptr) {
    std::memset(version, 0, sizeof(*version));
  }
}

double HAL_GetREVPHVoltage(HAL_REVPHHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_REVPHHandle HAL_InitializeREVPH(int32_t busId, int32_t module, const char* allocationLocation, int32_t* status) {
  *status = NotSupported("REV PH");
  return HAL_INVALID_HANDLE;
}

void HAL_SetREVPHClosedLoopControlAnalog(HAL_REVPHHandle handle, double minAnalogVoltage, double maxAnalogVoltage, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_SetREVPHClosedLoopControlDigital(HAL_REVPHHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_SetREVPHClosedLoopControlDisabled(HAL_REVPHHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_SetREVPHClosedLoopControlHybrid(HAL_REVPHHandle handle, double minAnalogVoltage, double maxAnalogVoltage, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_SetREVPHCompressorConfig(HAL_REVPHHandle handle, const HAL_REVPHCompressorConfig* config, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_SetREVPHSolenoids(HAL_REVPHHandle handle, int32_t mask, int32_t values, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

// ---- prosdk/hal/PowerDistribution.h ----

HAL_Bool HAL_CheckPowerDistributionChannel(HAL_PowerDistributionHandle handle, int32_t channel) {
  return 0;
}

HAL_Bool HAL_CheckPowerDistributionModule(int32_t module, HAL_PowerDistributionType type) {
  return 0;
}

void HAL_CleanPowerDistribution(HAL_PowerDistributionHandle handle) {

}

void HAL_ClearPowerDistributionStickyFaults(HAL_PowerDistributionHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_FreePowerDistributionStreamData(HAL_PowerDistributionChannelData* data, int32_t count) {}

void HAL_GetPowerDistributionAllChannelCurrents(HAL_PowerDistributionHandle handle, double* currents, int32_t currentsLength, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  if (currents != nullptr) {
    *currents = 0;
  }
}

double HAL_GetPowerDistributionChannelCurrent(HAL_PowerDistributionHandle handle, int32_t channel, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

void HAL_GetPowerDistributionFaults(HAL_PowerDistributionHandle handle, HAL_PowerDistributionFaults* faults, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  if (faults != nullptr) {
    std::memset(faults, 0, sizeof(*faults));
  }
}

int32_t HAL_GetPowerDistributionModuleNumber(HAL_PowerDistributionHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

int32_t HAL_GetPowerDistributionNumChannels(HAL_PowerDistributionHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

void HAL_GetPowerDistributionStickyFaults(HAL_PowerDistributionHandle handle, HAL_PowerDistributionStickyFaults* stickyFaults, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  if (stickyFaults != nullptr) {
    std::memset(stickyFaults, 0, sizeof(*stickyFaults));
  }
}

HAL_PowerDistributionChannelData* HAL_GetPowerDistributionStreamData(HAL_PowerDistributionHandle handle, int32_t* count, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  if (count != nullptr) {
    *count = 0;
  }
  return nullptr;
}

HAL_Bool HAL_GetPowerDistributionSwitchableChannel(HAL_PowerDistributionHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

double HAL_GetPowerDistributionTemperature(HAL_PowerDistributionHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

double HAL_GetPowerDistributionTotalCurrent(HAL_PowerDistributionHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

double HAL_GetPowerDistributionTotalEnergy(HAL_PowerDistributionHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

double HAL_GetPowerDistributionTotalPower(HAL_PowerDistributionHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_PowerDistributionType HAL_GetPowerDistributionType(HAL_PowerDistributionHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return HAL_POWER_DISTRIBUTION_AUTOMATIC;
}

void HAL_GetPowerDistributionVersion(HAL_PowerDistributionHandle handle, HAL_PowerDistributionVersion* version, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  if (version != nullptr) {
    std::memset(version, 0, sizeof(*version));
  }
}

double HAL_GetPowerDistributionVoltage(HAL_PowerDistributionHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_PowerDistributionHandle HAL_InitializePowerDistribution(int32_t busId, int32_t moduleNumber, HAL_PowerDistributionType type, const char* allocationLocation, int32_t* status) {
  *status = NotSupported("Power distribution (PDP/PDH)");
  return HAL_INVALID_HANDLE;
}

void HAL_ResetPowerDistributionTotalEnergy(HAL_PowerDistributionHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_SetPowerDistributionSwitchableChannel(HAL_PowerDistributionHandle handle, HAL_Bool enabled, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_StartPowerDistributionStream(HAL_PowerDistributionHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_StopPowerDistributionStream(HAL_PowerDistributionHandle handle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

}  // extern "C"
