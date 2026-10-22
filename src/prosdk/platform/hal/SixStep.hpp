// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Six-step (trapezoidal) BLDC commutation from 3 Hall sensors (design/boardoza/P15-drv8317h.md),
// for three-phase gate drivers in "3x PWM" mode such as the TI DRV8317H: per phase one PWM input
// INHx (high or low side) and one enable INLx (low: the half-bridge is Hi-Z). Every Hall edge
// must switch the phases within microseconds, which neither the robot loop nor a thread can do,
// and WPILib has no interrupt HAL; so this is a prosdk extension like PulseIO and SPI: plain C++
// functions, not a HAL_* C API. User: prosdk::DRV8317H.
//
// ESP32: esp32/SixStep.cpp. A GPIO interrupt on each Hall pin reads the 3 Hall levels and
// switches the 6 outputs from a precomputed 8-entry table with register writes only. One MCPWM
// generator makes the PWM; the interrupt routes it through the GPIO matrix to the one INHx pin
// that switches, the other INHx pins are held low. All 6 outputs are HAL output pins: closed
// output gate (robot disabled) = coast, emergency stop = all six low for good. Host:
// SixStepSim.cpp, which commutates on DIO sim value changes of the Hall pins.

#pragma once

#include <stdint.h>

#include <array>
#include <cstddef>

namespace prosdk::platform::hal {

/** What one half-bridge is driven to. */
enum class SixStepPhase : uint8_t {
  kFloat = 0,  ///< INLx low: Hi-Z
  kLow = 1,    ///< INLx high, INHx low: low side on
  kPwm = 2,    ///< INLx high, INHx = PWM: high side on for the duty, low side for the rest
};

/**
 * The phase pair driven for one Hall state: `high` gets the PWM, `low` is held low, the third
 * phase floats. Phases are 0 = A, 1 = B, 2 = C; {-1, -1} marks a Hall state that is never valid
 * (all phases float there).
 */
struct SixStepDrive {
  int8_t high = -1;
  int8_t low = -1;

  constexpr bool operator==(const SixStepDrive&) const = default;
};

/**
 * Commutation table indexed by the Hall state A | B << 1 | C << 2 (a high Hall input is 1). It
 * holds the drive for the positive (forward) direction; the reverse direction swaps `high` and
 * `low`. Exactly six entries are valid and use six different phase pairs; the other two are
 * {-1, -1} (states 0 and 7 for 120-degree sensors, 2 and 5 for 60-degree sensors).
 */
using SixStepTable = std::array<SixStepDrive, 8>;

/**
 * The table of the Boardoza DRV8317 Arduino library (repository "software files/src", BSD-3
 * licensed): 120-degree sensors, states 1, 3, 2, 6, 4, 5 drive C+A-, C+B-, A+B-, A+C-, B+C-,
 * B+A-. The right table depends on the motor's sensor and winding order: unverified on a motor.
 */
inline constexpr SixStepTable kDefaultSixStepTable = {{
    {-1, -1},  // 0: invalid
    {2, 0},    // 1: A      -> C+ A-
    {0, 1},    // 2: B      -> A+ B-
    {2, 1},    // 3: A B    -> C+ B-
    {1, 2},    // 4: C      -> B+ C-
    {1, 0},    // 5: A C    -> B+ A-
    {0, 2},    // 6: B C    -> A+ C-
    {-1, -1},  // 7: invalid
}};

/** GPIOs of one motor. All 9 must be different free GPIOs. */
struct SixStepPins {
  std::array<int32_t, 3> inh{-1, -1, -1};   ///< INHA, INHB, INHC (PWM inputs)
  std::array<int32_t, 3> inl{-1, -1, -1};   ///< INLA, INLB, INLC (enables)
  std::array<int32_t, 3> hall{-1, -1, -1};  ///< Hall A, B, C (inputs with pull-up)
};

/** Hall feedback and output state of one motor. */
struct SixStepFeedback {
  int32_t hallState = 0;      ///< last Hall state read, A | B << 1 | C << 2
  int64_t steps = 0;          ///< counted Hall steps, positive in the table's forward direction
  int64_t lastStepUs = 0;     ///< time of the last counted step (clock of nowUs)
  int64_t periodUs = 0;       ///< signed time per step between the last two steps (0: none yet)
  int64_t nowUs = 0;          ///< the clock (HAL_GetMonotonicTime / 1000 on the host) at this read
  uint32_t badStates = 0;     ///< Hall reads the table does not drive, or that skipped 3 steps
  std::array<SixStepPhase, 3> phases{};  ///< what A, B and C are driven to now
};

/** Handle value of no motor. */
inline constexpr int32_t kInvalidSixStepHandle = 0;

/** Motors that can be open at once (each takes an MCPWM timer and operator on the ESP32). */
inline constexpr int32_t kMaxSixStepMotors = 2;

/** PWM frequency range of OpenSixStep. */
inline constexpr uint32_t kMinSixStepPwmHz = 1000;
inline constexpr uint32_t kMaxSixStepPwmHz = 100000;

/** MCPWM counter clock: the PWM period is kSixStepResolutionHz / pwmHz ticks. */
inline constexpr uint32_t kSixStepResolutionHz = 10000000;

/**
 * Opens a motor. All outputs start in coast (every INLx and INHx low) with the table
 * kDefaultSixStepTable.
 *
 * @param pins               The 9 GPIOs.
 * @param pwmHz              PWM frequency, kMinSixStepPwmHz to kMaxSixStepPwmHz.
 * @param allocationLocation Shown when a pin is allocated twice (may be nullptr).
 * @param status             HAL status: HAL_PARAMETER_OUT_OF_RANGE (frequency), HAL_RESOURCE_*
 *                           (a pin twice, in use or not a GPIO), HAL_NO_AVAILABLE_RESOURCES (no
 *                           motor slot or MCPWM timer left), HAL_INCOMPATIBLE_STATE after an
 *                           emergency stop.
 * @return The handle, or kInvalidSixStepHandle.
 */
int32_t OpenSixStep(const SixStepPins& pins, uint32_t pwmHz, const char* allocationLocation,
                    int32_t* status);

/** Replaces the commutation table (HAL_PARAMETER_OUT_OF_RANGE when CheckSixStepTable fails). */
void SetSixStepTable(int32_t handle, const SixStepTable& table, int32_t* status);

/**
 * Sets the output. A duty of -1 to 1 (clamped; NaN is 0) drives in the table's direction
 * (positive) or the reverse one. A duty that rounds to 0 PWM ticks brakes (`brake`: all three
 * low sides on) or coasts (all Hi-Z). The ESP32 backend coasts while the output gate is closed
 * and ignores this call after an emergency stop.
 */
void SetSixStepOutput(int32_t handle, double duty, bool brake, int32_t* status);

/** Reads the Hall feedback. Returns false for an invalid handle. */
bool GetSixStepFeedback(int32_t handle, SixStepFeedback* out, int32_t* status);

/** Coasts and closes a motor; frees its pins. */
void CloseSixStep(int32_t handle);

namespace detail {

/**
 * Index of a phase pair in rotation order: A+B-, A+C-, B+C-, B+A-, C+A-, C+B- are 0 to 5, so a
 * motor turning in the direction a table drives steps +1 per Hall edge. -1 for no valid pair.
 */
constexpr int StepOf(SixStepDrive d) {
  constexpr SixStepDrive kOrder[6] = {{0, 1}, {0, 2}, {1, 2}, {1, 0}, {2, 0}, {2, 1}};
  for (int i = 0; i < 6; ++i) {
    if (kOrder[i] == d) {
      return i;
    }
  }
  return -1;
}

/** nullptr when the table is usable, else the reason. */
constexpr const char* CheckSixStepTable(const SixStepTable& table) {
  int valid = 0;
  bool used[6] = {};
  for (const SixStepDrive& d : table) {
    if (d == SixStepDrive{}) {
      continue;
    }
    int step = StepOf(d);
    if (step < 0) {
      return "six-step: each table entry must be two different phases (0-2) or {-1, -1}";
    }
    if (used[step]) {
      return "six-step: the table drives one phase pair for two Hall states";
    }
    used[step] = true;
    ++valid;
  }
  return valid == 6 ? nullptr : "six-step: the table must drive exactly six Hall states";
}

/** Effective output mode (duty sign or zero-duty behaviour). */
enum class SixStepMode : uint8_t { kCoast, kBrake, kForward, kReverse };

/** The phases for a Hall state in a mode. */
constexpr std::array<SixStepPhase, 3> PhasesFor(const SixStepTable& table, int hallState,
                                                SixStepMode mode) {
  std::array<SixStepPhase, 3> out{SixStepPhase::kFloat, SixStepPhase::kFloat,
                                  SixStepPhase::kFloat};
  if (mode == SixStepMode::kBrake) {
    out = {SixStepPhase::kLow, SixStepPhase::kLow, SixStepPhase::kLow};
  } else if (mode != SixStepMode::kCoast) {
    SixStepDrive d = table[static_cast<size_t>(hallState & 7)];
    if (StepOf(d) >= 0) {
      if (mode == SixStepMode::kReverse) {
        d = {d.low, d.high};
      }
      out[static_cast<size_t>(d.high)] = SixStepPhase::kPwm;
      out[static_cast<size_t>(d.low)] = SixStepPhase::kLow;
    }
  }
  return out;
}

/** Steps from one table step index (0-5) to another: -2 to 2, or 3 when ambiguous. */
constexpr int StepDelta(int from, int to) {
  int d = ((to - from) % 6 + 6) % 6;
  return d <= 2 ? d : (d == 3 ? 3 : d - 6);
}

/** PWM compare ticks for a duty, at most the period minus one tick (so at most 99.x %). */
constexpr uint32_t DutyTicks(double duty, uint32_t periodTicks) {
  double magnitude = duty < 0.0 ? -duty : duty;
  if (!(magnitude > 0.0)) {
    return 0;
  }
  if (magnitude > 1.0) {
    magnitude = 1.0;
  }
  auto ticks = static_cast<uint32_t>(magnitude * periodTicks + 0.5);
  return ticks >= periodTicks ? periodTicks - 1 : ticks;
}

}  // namespace detail

}  // namespace prosdk::platform::hal
