// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../PortsInternal.hpp"
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
#include "prosdk/hal/simulation/PWMData.h"
#include "prosdk/hal/simulation/PowerDistributionData.h"
#include "prosdk/hal/simulation/REVPHData.h"
#include "prosdk/hal/simulation/RoboRioData.h"
#include "prosdk/hal/simulation/SimDeviceData.h"
#include "prosdk/util/Alert.h"

extern "C" void HALSIM_ResetAllSimData(void) {
  for (int32_t i = 0; i < prosdk::hal::NUM_ADDRESSABLE_LE_DS; i++) {
    HALSIM_ResetAddressableLEDData(i);
  }

  for (int32_t i = 0; i < prosdk::hal::NUM_ANALOG_INPUTS; i++) {
    HALSIM_ResetAnalogInData(i);
  }

  HALSIM_ResetCanData();

  for (int32_t i = 0; i < prosdk::hal::NUM_CTREPCM_MODULES; i++) {
    HALSIM_ResetCTREPCMData(i);
  }

  for (int32_t i = 0; i < prosdk::hal::NUM_DIGITAL_PWM_OUTPUTS; i++) {
    HALSIM_ResetDigitalPWMData(i);
  }

  for (int32_t i = 0; i < prosdk::hal::NUM_DIGITAL_CHANNELS; i++) {
    HALSIM_ResetDIOData(i);
  }

  HALSIM_ResetDriverStationData();

  for (int32_t i = 0; i < prosdk::hal::NUM_DUTY_CYCLES; i++) {
    HALSIM_ResetDutyCycleData(i);
  }

  for (int32_t i = 0; i < prosdk::hal::NUM_ENCODERS; i++) {
    HALSIM_ResetEncoderData(i);
  }

  for (int32_t i = 0; i < prosdk::hal::I2C_PORTS; i++) {
    HALSIM_ResetI2CData(i);
  }

  for (int32_t i = 0; i < prosdk::hal::NUM_PD_SIM_MODULES; i++) {
    HALSIM_ResetPowerDistributionData(i);
  }

  for (int32_t i = 0; i < prosdk::hal::NUM_PWM_CHANNELS; i++) {
    HALSIM_ResetPWMData(i);
  }

  for (int32_t i = 0; i < prosdk::hal::NUM_REVPH_MODULES; i++) {
    HALSIM_ResetREVPHData(i);
  }

  HALSIM_ResetRoboRioData();
  HALSIM_ResetSimDeviceData();
  WPI_ResetAlertData();
}
