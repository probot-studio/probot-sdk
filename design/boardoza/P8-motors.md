# P8: brushed and BLDC motor drivers

Package P8 of the Boardoza plan. It adds three motor controller classes, written like
`prosdk::BTS7960` and `prosdk::VNH5019`:

- `prosdk::TwoPinHBridge`, with the aliases `prosdk::L9110S` and `prosdk::BA6208`;
- `prosdk::VNH7070AS`;
- `prosdk::DRV8307`.

The DRV8317H board is package P15 and is not covered here. The VNH5019 and BTS7960 files are
unchanged.

| Board | Ozdisan id | Class | Repo (`Boardoza_` prefix omitted) |
|---|---|---|---|
| L9110S dual DC motor driver | 1564784 | `L9110S` (= `TwoPinHBridge`), one per channel | L9110S_Dual_DC_Motor_Driver_Breakout_Board |
| BA6208 motor driver | 1065566 | `BA6208` (= `TwoPinHBridge`) | BA6208_Motor_Driver_Breakout_Board |
| VNH7070AS bidirectional DC motor driver | 1598829 | `VNH7070AS` | VNH7070AS_Bidirectional_DC_Motor_Driver_Breakout_Board |
| DRV8307 48W BLDC motor driver | 1473936 | `DRV8307` | none (Ozdisan only) |

Sources:
- BA6208: UTC BA6208 datasheet QW-R109-004.G (the copy in the repo's `assets/`).
- L9110S: the L9110 datasheet in the repo's `assets/`, and the KiCad PCB netlist
  `B-L9110S-Brk-R1.0.kicad_pcb`.
- VNH7070AS: ST datasheet DocID028088 Rev 5 (in the repo's `assets/`), and the KiCad schematic
  `B-VNH7070AS-BRK-R1.0.kicad_sch`.
- DRV8307: TI datasheet SLVSCK2A, and the Ozdisan product page and photo.

Anything not checked against a datasheet or a schematic is marked **unverified**.

Rules for all four boards:
- ESP32-S3 pins are 3.3 V only and **not 5 V tolerant**. Every board below has a motor supply
  above 3.3 V, and some of them can put it on a logic pin. Read the hazards before wiring.
- Tie the host GND to the board's power GND.
- All three classes are `MotorController`, `MotorSafety` and `TelemetryLoggable`, as BTS7960 and
  VNH5019 are. Each one has:
  - `SetThrottle(-1..1)`, which clamps the value (NaN is 0) and feeds the watchdog;
  - `GetThrottle`, `SetInverted` and `GetInverted`;
  - `Disable()` and `StopMotor()`;
  - `GetDescription()`;
  - telemetry of type "Motor Controller" with the entry "Value";
  - a `SimDevice` with a "Throttle" value (after inversion) for tests.
- The PWM uses the HAL digital PWM (`DigitalOutput::EnablePWM`, the LEDC). It has 6 generators and
  **one rate for all of them**. Every class here sets that rate to 20 kHz in its constructor,
  as BTS7960 and VNH5019 do (plan G10).
- While the robot is disabled the PWM duty is forced to 0. Plain DIO levels are kept, as on the
  roboRIO. After an emergency stop every output pin is driven low. Each class section below
  says what these two states mean for its chip.

## 1. `prosdk::TwoPinHBridge` (L9110S, BA6208)

Files: `src/prosdk/drivers/motor/TwoPinHBridge.hpp` and `.cpp`.

```cpp
TwoPinHBridge(int inA, int inB, ZeroBehavior zeroBehavior = ZeroBehavior::kBrake);
void SetZeroBehavior(ZeroBehavior);   // kCoast or kBrake
ZeroBehavior GetZeroBehavior() const;
using L9110S = TwoPinHBridge;
using BA6208 = TwoPinHBridge;
```

How the inputs are driven:
- Both inputs carry a PWM generator.
- Forward: PWM = throttle on A, and B low.
- Reverse: PWM on B, and A low.
- When the direction changes, the inactive input is set low first. Starting from H/H, the
  bridge therefore passes only through the target direction, never through the opposite one.
- Throttle 0: both inputs high with `kBrake` (the default), or both low with `kCoast`.
- `Disable()` and `StopMotor()`: both inputs low.
- PWM off-time: both inputs low.

Truth tables (the rows were checked against each datasheet):

| A | B | BA6208 (QW-R109-004.G p.3) | L9110 (datasheet in the L9110S repo) |
|---|---|---|---|
| H | L | forward | forward |
| L | H | reverse | reverse |
| H | H | brake (both outputs low) | both outputs low (brake) |
| L | L | both outputs open (coast, standby current at most 0.4 mA) | both outputs low (brake) |

What this means for each chip:
- **BA6208**: the PWM off-time, `kCoast`, `Disable()`, a closed output gate and an emergency stop
  all coast. `kBrake` brakes.
- **L9110**: the datasheet has no coast state. Everything above brakes, and `kCoast` behaves like
  `kBrake`. Some L9110 datasheets from other vendors show L/L as open. **Unverified** on the
  chips actually fitted to the Boardoza board.

### L9110S board wiring

J1 (pin header) and J4 (locking connector) are wired in parallel:

| Pin | Signal |
|---|---|
| 1 | VCC: motor and chip supply, 2.5-12 V |
| 2 | M2_B |
| 3 | M2_A |
| 4 | M1_B |
| 5 | M1_A |
| 6 | GND |

Use one class per channel:

```cpp
prosdk::L9110S left{m1a, m1b};
prosdk::L9110S right{m2a, m2b};
```

- The inputs need VIH of at least 2.5 V (datasheet, at Vcc = 9 V). 3.3 V logic meets this, with
  a 0.8 V margin.
- **5 V hazard (verified from the PCB netlist):** R1-R4 are 10 kOhm pull-ups from all four inputs
  to **VCC, the motor supply**.
  - With VCC above 3.3 V, every input the ESP32 drives high, or leaves floating, is pulled toward
    VCC.
  - At 12 V that is roughly 0.8 mA into each GPIO's protection diode. The ESP32-S3 pins are
    rated to VDD + 0.3 V.
  - Use one of these fixes: remove R1-R4; put a level shifter between the ESP32 and the inputs;
    or run VCC at 3.3 V (the motors then see about 1.4 V).
  - With R1-R4 removed, the inputs float until the constructor drives them low. What the chip does
    with floating inputs is **unverified**.
- Undriven inputs with R1-R4 fitted read H/H, which is brake.
- Current: 750-850 mA continuous per channel, 1.5-2 A peak. At 9 V and 750 mA the output stage
  drops about 1.9 V (VOH 7.6 V, VOL 0.45 V). There is no current sense, no fault output and no
  enable pin.

### BA6208 board wiring

J1 pins: 1 "+5V" (VCC, 4.5-15 V, motor and chip supply), 2 A IN, 3 B IN, 4 GND. Motor on M1.

- The inputs are TTL: VIH at least 2.0 V, VIL at most 0.8 V. 3.3 V GPIOs drive them directly.
- The J1 pin is labelled "+5V", but the README gives 4.5-15 V. Whether it feeds VCC directly is
  **unverified**, because the schematic is not in the repo.
- Whether the board has pull-ups or pull-downs on AIN/BIN is **unverified**. If it pulls them up
  to VCC, as the L9110S board does, the same 5 V hazard applies. Measure before you connect.
- Output current: 100 mA typical, 200 mA minimum (electrical table), 500 mA absolute maximum.
  The saturation drop is up to 1.6 V at 100 mA. It suits only very small motors.
- The README's "thermal protection" is not in the datasheet (**unverified**).

### Limits and defaults

- Neither datasheet gives a maximum PWM frequency. Both chips are bipolar. Clean switching and the
  losses at 20 kHz are **unverified** and need a bench test. The Boardoza examples use
  1-10 kHz. The rate cannot be lowered for one driver alone: it is shared with every digital PWM
  output.
- Uses 2 PWM generators per motor, so the dual L9110S board takes 4 of the 6.

## 2. `prosdk::VNH7070AS`

Files: `src/prosdk/drivers/motor/VNH7070AS.hpp` and `.cpp`.

```cpp
VNH7070AS(int ina, int inb, int pwm, int sel0 = -1, int csAnalog = -1);
void SetBrakeMode(bool brake);            // default true
void SetBrakeStrength(double strength);   // 0..1, default 1
void SetCurrentSense(units::ohm_t senseResistance, double dividerRatio = 1.0);
units::ampere_t GetCurrent() const;
bool IsFault() const;
static constexpr double kSenseRatio = 1540.0;
static constexpr units::ohm_t kBoardSenseResistance{1500.0};
static constexpr double kFaultSenseVoltage = 5.0;
```

Control is the same as `prosdk::VNH5019` (Table 11):
- Forward: INA high, INB low, PWM = throttle. Reverse is the opposite.
- Throttle 0 with brake mode on: INA = INB = low, PWM = brake strength. Both low sides are on.
- Throttle 0 with brake mode off: PWM 0 (coast).
- `Disable()`: PWM 0 and INA = INB = SEL0 = low (coast).
- When INA/INB change, the PWM is set to 0, the pins switch, and the PWM rises 20 us later. This
  is a busy wait, and it follows the order the datasheet gives for leaving standby (Table 6,
  note 1). Braking from standby (coast, then a throttle of 0 in brake mode, or re-enabling the
  robot while braking) raises the PWM with INA = INB = 0, so it cannot follow that order; the
  note recommends the order only to avoid stress on an existing short to battery.

SEL0 and CS:
- SEL0 is driven to the INA level. CS then reports the HSDA current when going forward and the
  HSDB current in reverse (Table 11). The Boardoza Arduino library leaves SEL0 high, so its
  reverse readings are Hi-Z.
- With `sel0 = -1` you wire SEL0 yourself. `GetCurrent()` is then valid in one direction only:
  forward with SEL0 high, reverse with SEL0 low. In the other direction CS is Hi-Z. What a floating
  SEL0 does is **unverified**.
- Standby: INA = INB = SEL0 = PWM = 0 puts the chip in standby after tD_sdby, 0.2-1.8 ms. Its
  supply current is then at most 3 uA (Table 6). Coast at 0, `Disable()` and an emergency stop
  all reach this state.
- `GetCurrent()` = CS voltage / sense resistance x K, where the CS voltage is the ADC voltage /
  divider ratio and K = 1540. It returns 0 A without a CS pin.
  - K is typical at 3.5 A and 5.5 A: K2/K3 1540, with a min-max of 1315-1779 (Table 10).
  - Below about 0.2 A K is not linear: K1 is 1083-2716 and K0 is 665. Readings there are rough.
  - While braking or coasting CS is Hi-Z, so the reading is about 0.
- `IsFault()` is true when the CS voltage is at least 5 V (VSENSEH 5-7 V, Tables 10 and 12) while
  the bridge drives or brakes. It is false:
  - while coasting or disabled, because CS then shows the off-state diagnostic (Table 13), which
    a spinning motor can also trigger;
  - while braking with the output gate closed;
  - always without a divider, because the ADC cannot read 5 V.
- What `IsFault()` can see (Table 12): with SEL0 = INA, a latched high side of the driving leg
  (HSA forward, HSB reverse) and, while braking, LSB. A latched low side while driving is not
  reported, because those rows need SEL0 opposite to INA.
- A latched fault is cleared by toggling the inputs (tLATCH_RST 3-20 us, Table 9). `Disable()`,
  then `SetThrottle()` at least 20 us later (for example in the next loop iteration), does this.
  Called back to back, the inputs may be low for less than 20 us.
- Telemetry adds "Current" (A) when a CS pin is given.

### Board wiring

J1 pins: 1 INA, 2 INB, 3 SEL0, 4 PWM, 5 CS. INA, INB, SEL0 and PWM each pass through a 1 kOhm
series resistor on the board.

- **J1 has no GND.** Tie the host GND to the power terminal's GND.
- Terminals by schematic reference (the silkscreen was not checked): J3 is power, pin 1 VIN
  4-28 V (the chip's absolute maximum is 41 V), pin 2 GND; J2 is the motor, pin 1 OUTA, pin 2
  OUTB. The board has reverse-polarity
  protection (a P-MOSFET) and 270 uF of bulk capacitance. The voltage rating of that capacitor is
  not given in the schematic, and it may limit VIN (**unverified**).
- Logic is 3 V CMOS compatible: VIL at most 0.9 V, VIH at least 2.1 V (Table 7). 3.3 V GPIOs
  drive it directly.
- CS network (values and nets traced from the schematic's wires, labels and pin positions):
  - R5 = 1.5 kOhm from the chip's CS pin to GND;
  - R4 = 10 kOhm in series from the chip's CS pin to J1 pin 5;
  - C3 = 33 nF from J1 pin 5 to GND.
- With this network CS gives about 0.97 V/A (1500 / 1540). The ESP32 ADC reads up to about
  3.1 V, so **without a divider the reading saturates near 3.2 A**.
- **CS voltage hazard:** CS can exceed 3.3 V. Above about 3.4 A the analog CS voltage passes 3.3 V
  (VSENSESAT is at least 5 V). In a fault CS sits at 5-7 V (VSENSEH). R4 limits the current into
  the ESP32 pin to roughly 0.2-0.4 mA (computed).
- To read faults, and to stay below 3.3 V, add a resistor Rb from the J1 CS pin to GND. It forms a
  divider with R4. Example, with Rb = 6.8 kOhm:
  - divider ratio = 6.8 / (10 + 6.8) = 0.405;
  - sense resistance = 1.5k parallel to (10k + 6.8k) = 1.377 kOhm;
  - call `SetCurrentSense(units::ohm_t{1377.0}, 0.405)`;
  - 5-7 V at CS then reads 2.0-2.8 V, and about 5.5 A reads 2.0 V.
  - Above about 5.6 A the CS voltage reaches 5 V, so `IsFault()` cannot tell a current that
    high from a fault.
  - Check these values on the board before you rely on them.
- Thermal (README): the chip is rated 15 A, but the board heats up fast at 5 A. Treat 5 A as a
  short-duration maximum.

### Limits and defaults

- **20 kHz is the VNH7070AS maximum** (fPWM 0-20 kHz, Table 8). Never raise the shared
  digital PWM rate while this driver is in use.
- Uses 1 PWM generator, 2-3 GPIO outputs and optionally 1 ADC1 pin (GPIO 1-10).
- While the robot is disabled the PWM is 0 and INA/INB are kept. The active high side stays on,
  so the motor freewheels, as with VNH5019. In brake mode the bridge is off.

## 3. `prosdk::DRV8307`

Files: `src/prosdk/drivers/motor/DRV8307.hpp` and `.cpp`.

```cpp
DRV8307(int pwm, int enableN, int dir, int brake, int lockN = -1, int hallOut = -1,
        int faultN = -1);
void SetBrakeMode(bool brake);   // default true
bool IsLocked() const;           // LOCKn low
bool IsFault() const;            // FAULTn low
void SetPolePairs(int polePairs);
units::revolutions_per_minute_t GetRpm() const;
```

The DRV8307 is a sensored three-phase BLDC controller. It commutates from the motor's Hall
sensors by itself. The class uses clock PWM mode: the speed command is the PWM duty cycle, which
sets the output duty (open loop). The datasheet facts below are from SLVSCK2A.

How the pins are driven:
- Non-zero throttle: DIR high for a positive throttle and low for a negative one, BRAKE low,
  PWM duty = |throttle|, then ENABLEn low.
- Throttle 0: ENABLEn high, BRAKE high (brake mode, the default) or low (coast), and the PWM at 0.
  - The datasheet says a driving motor should be stopped with ENABLEn, not with 0 % duty
    (section 7.4.1).
  - Holding BRAKE high together with ENABLEn makes it stop faster and lets it be driven again
    sooner (section 8.1.3). BRAKE = 1 brakes (Table 2).
- `Disable()`, and the constructor: ENABLEn high, BRAKE low, DIR low, PWM 0 (coast).
- `dir = -1` or `brake = -1` is allowed when the board's DIR switch or BRAKE jumper sets that
  input. Without DIR, negative throttles stop the motor. Without BRAKE, a throttle of 0 coasts.

What the chip does on its own:
- **Restart delay.** Once ENABLEn has gone high, the chip runs its stop-and-power-down sequence to
  the end (sections 7.3.3 and 8.1.2):
  - the motor coasts, or brakes;
  - the chip enters standby 1 s after the last HALLOUT edge;
  - the motor cannot be driven again until it has stopped.
  - A closed loop that sets exactly 0 now and then therefore stalls for more than a second each
    time. Avoid an exact 0 inside a loop.
- **Reversing.** Reversing a spinning motor makes the chip coast until the Hall U period exceeds
  160 ms. Then it drives in the new direction (section 7.3.4).
- **Locked rotor.** With no HALLOUT edges for 3 s while enabled, or with BRAKE high for more than
  3 s while the PWM clock runs, the chip reports a locked rotor (section 7.3.11.6). FAULTn goes low
  and the chip retries after 5 s. The class stops with ENABLEn high, so an idle motor does not
  trip this.
  - While the robot is disabled with a non-zero throttle set, ENABLEn stays low (DIO is not gated)
    and the duty is 0. After 3 s this can report a locked rotor that clears itself. The
    `MotorSafety` watchdog, when enabled, stops the motor first.
- **FAULTn** goes low on undervoltage, overvoltage, overcurrent (retry after 5 ms), a charge
  pump failure or short, a locked rotor or overtemperature (retry after 5 s), section 7.3.11.
  `IsFault()` returns it, and is false without the pin.
- **LOCKn** goes low once the speed is steady. The chip then commutates on Hall U only
  (section 7.3.4.2). `IsLocked()` returns it, and is false without the pin.
- **HALLOUT** is a copy of Hall U (section 7.3.2, Figure 4): one period per electrical
  revolution.
  - `GetRpm()` reads it with a `Tachometer` (rising edges, one PCNT unit), with
    `SetPolePairs(p)` edges per revolution. The default of 1 gives electrical RPM.
  - The value has no sign.
  - Resolution is one edge per rate window: 20 Hz with the HAL's default 50 ms window, that is
    1200 / p RPM steps. That is coarse at low speed.

### Board wiring

There is no repo, schematic or board datasheet. Everything here comes from the Ozdisan photo and
is **unverified**:
- J6 is GND / VIN, 8.5-32 V (the chip's operating range, section 6.3).
- J11 is the U/V/W motor phases.
- J2 is the Hall connector (HPWR, HU+/-, HV+/-, HW+/-, HGND). J4 selects 5 V Hall power from
  VREG.
- Test pads: Enable, HallOut, FaultN, LockN. J5 is the PWM input pad.
- J14 selects the PWM source: external, or the onboard 555 timer (speed pot R4). **Set J14 to the
  external PWM.** With the 555 selected, the motor runs whatever the ESP32 does.
- DIR is the slide switch S1 and BRAKE is the jumper J9. Whether either reaches a header pin is
  unknown. Pass -1 for them, or wire to the pads if they are exposed.

Logic levels:
- Inputs: VIH at least 1.5 V, absolute maximum 5.75 V. 3.3 V is fine.
- ENABLEn has a 350-550 kOhm internal pull-down. DIR, BRAKE and PWM have 50-150 kOhm pull-downs.
- **A floating ENABLEn means enabled.** Before the constructor runs, and after an emergency stop
  (every pin driven low), the chip is enabled at 0 % duty. The motor is not driven, because the
  PWM pin is low, as long as J14 selects the external PWM.

The HALLOUT, FAULTn and LOCKn outputs are open drain:
- `DigitalInput` and the counter input use the ESP32 internal pull-up to 3.3 V.
- **If the board pulls them up to 5 V (VREG), they are unsafe for the ESP32.** Measure first
  (**unverified**).

Other unverified items: the current limit (set by the board's RISENSE), the FET part numbers and
the basis of the "48W" rating.

### Limits and defaults

- PWM input range: 16-50 kHz (section 6.3). 20 kHz is set, and the gate driver itself switches at
  a fixed 25 kHz.
- Uses 1 PWM generator, 1-3 GPIO outputs, up to 2 GPIO inputs, and optionally 1 PCNT unit for
  HALLOUT. The 4 PCNT units are shared with encoders and counters.

## 4. Resource budget

| Driver | PWM generators | Other |
|---|---|---|
| TwoPinHBridge (per motor) | 2 | - |
| L9110S board (2 motors) | 4 | - |
| VNH7070AS | 1 | 2-3 DIO out, ADC1 pin for CS (optional) |
| DRV8307 | 1 | 1-3 DIO out, 0-2 DIO in, PCNT unit for HALLOUT (optional) |
| BTS7960 (existing) | 2 (`PWM_ON_INPUTS`) or 1 | 1 DIO out |
| VNH5019 (existing) | 1 | 2-4 DIO out |

There are 6 PWM generators in total, all at one rate. The last driver constructed sets the rate,
and every driver here sets 20 kHz.

## 5. Verified and unverified

Verified against the datasheets:
- The BA6208 truth table, including coast on L/L and brake on H/H, and its TTL input levels.
- The L9110 truth table in the repo's datasheet: L/L and H/H both give outputs low. Also its input
  levels, its current, and the input pull-ups to VCC (PCB netlist).
- VNH7070AS:
  - the truth tables 11, 12 and 13 (SEL0 addressing, fault CS, off-state diagnostic);
  - the standby entry and exit order (Table 6);
  - fPWM up to 20 kHz (Table 8);
  - K 1540 typ and VSENSEH 5-7 V (Table 10);
  - tLATCH_RST (Table 9);
  - the board resistor and capacitor values and the CS network (schematic, net-traced), the J1
    pinout and the 1 kOhm series resistors.
- DRV8307:
  - PWM 16-50 kHz;
  - the ENABLEn, BRAKE and stop rules;
  - HALLOUT = Hall U (Figure 4);
  - the locked-rotor timing and LOCKn;
  - the logic levels and pull-downs.

Unverified:
- Whether L/L brakes or coasts on the L9110S parts actually fitted.
- The maximum PWM frequency of the L9110 and BA6208. Both are bipolar parts, and 20 kHz may run
  hot.
- The BA6208 board: its input pull resistors, and whether VCC is direct.
- The VNH7070AS board silkscreen (terminal labels were taken from schematic references), and the
  voltage rating of its 270 uF capacitor.
- The DRV8307 board: its pinout, J14, whether DIR and BRAKE are routed, the pull-up rail of the
  open-drain outputs, and its current limit.
- How long the DRV8307 restart takes after a stop.

## 6. Tests

Host tests, in the `prosdk_drivers_tests` executable, use the DIO, digital PWM, analog input and
Driver Station sims:

- `TwoPinHBridgeTest.cpp` (`[twopinhbridge]`):
  - "TwoPinHBridge direction, brake and coast": direction, H/H brake, the zero behavior setter,
    inversion and the SimDevice, clamping and NaN, `Disable`, and the description;
  - "TwoPinHBridge aliases and coast at zero": the alias types, `kCoast`, and `StopMotor`.
- `VNH7070ASTest.cpp` (`[vnh7070as]`):
  - "VNH7070AS direction, SEL0, brake and coast": SEL0 follows INA, brake strength, coast,
    inversion, and `Disable`;
  - "VNH7070AS current sense and fault": the current scaling with the default and a custom
    resistor and divider, the fault level while driving, and no fault while braking with the gate
    closed, while coasting, or after `Disable`;
  - "VNH7070AS without SEL0 and CS".
- `DRV8307Test.cpp` (`[drv8307]`):
  - "DRV8307 enable, direction, brake and coast": ENABLEn, DIR and BRAKE sequencing, inversion,
    `Disable`, LOCKn, FAULTn and the pole pairs;
  - "DRV8307 with DIR and BRAKE set on the board".

The sim counter always reports a rate of 0, so `GetRpm()` is tested only for 0. HALLOUT speed
measurement needs hardware.
