# DC Motor Characterisation and Closed-Loop Speed Control (STM32 + Simulink)

Characterisation and closed-loop speed control of a 12 V geared DC motor, using an **STM32 Nucleo-F446RE** programmed from **MATLAB/Simulink** and run live with **Monitor & Tune**.

The project covers eight tasks: motor characterisation (armature resistance, speed constant, starting current, load behaviour, efficiency), a PI speed controller, step-response analysis, and disturbance rejection with feed-forward and sensorless speed estimation.

<!-- Add a photo of the setup and a screenshot of the Simulink model here:
![Setup](docs/images/setup.jpg)
![Simulink model](docs/images/model.png)
-->

## Contents

- [Hardware](#hardware)
- [Wiring and pin map](#wiring-and-pin-map)
- [STM32CubeMX configuration](#stm32cubemx-configuration)
- [Simulink model](#simulink-model)
- [How to run](#how-to-run)
- [Tasks and results](#tasks-and-results)
- [Key formulas](#key-formulas)
- [Problems met and fixes](#problems-met-and-fixes)
- [Repository layout](#repository-layout)
- [Safety notes](#safety-notes)

## Hardware

| Part | Details |
| --- | --- |
| Controller | STM32 Nucleo-F446RE (STM32F446RE, 180 MHz) |
| Motor | GM25-370CA, 12 V, 170 rpm rated, 21:1 gearbox, 6-pin Hall encoder |
| Motor driver | L298N dual H-bridge |
| Supply | 12 V DC bench supply |
| Supply sensing | Resistor divider, 33 kΩ and 10 kΩ, into ADC |
| Current sensing | Shunt resistor (10 Ω) into ADC |
| Speed reference | Handheld tachometer |
| Load (Tasks 4 and 5) | Fan blades on the output shaft |

Encoder: 12 pulses per revolution on the motor shaft, counted in 4× quadrature, through a 21:1 gearbox, giving **1008 counts per output-shaft revolution**.

## Wiring and pin map

| Signal | Nucleo pin | Connects to | Purpose |
| --- | --- | --- | --- |
| PWM (TIM1_CH1) | PA8 (D7) | L298N **ENA** | Duty sets the average motor voltage |
| Direction 1 (GPIO) | PA9 (D8) | L298N **IN1** (set to 1) | Direction |
| Direction 2 (GPIO) | PC7 (D9) | L298N **IN2** (set to 0) | Direction |
| Encoder A (TIM4_CH1) | PB6 | Encoder C1 | Quadrature channel A |
| Encoder B (TIM4_CH2) | PB7 | Encoder C2 | Quadrature channel B |
| Encoder power | 3V3, GND | Encoder VCC, GND | 3.3 V logic |
| Supply sense (ADC1_IN0) | PA0 (A0) | Divider middle point | Measures the supply voltage |
| Motor | none | L298N OUT1, OUT2 | Motor terminals |
| Power | none | 12 V to L298N 12V terminal | Motor power |
| Common ground | GND | Supply minus, L298N GND, Nucleo GND | Shared reference |

Notes:

- **Remove the ENA jumper cap.** With the cap on, ENA is tied to 5 V and the PWM has no effect.
- **IN1 and IN2 must differ.** Equal levels brake the motor.
- Divider ratio is 10 / 43 = 0.2326, so 12 V reads as about 2.79 V at the ADC pin.

## STM32CubeMX configuration

| Peripheral | Setting | Reason |
| --- | --- | --- |
| Clock | 180 MHz (board default) | Timer source |
| TIM1 CH1 (PA8) | PWM Generation, prescaler 17, period 999 | 180 MHz / 18 / 1000 = **10 kHz**, 0.1% duty resolution |
| TIM4 (PB6, PB7) | Encoder mode TI1 and TI2, period 65535, input filter 5 | 4× counting on a 16-bit counter, glitch filtering |
| PA9, PC7 | GPIO output | Direction pins |
| ADC1 IN0 (PA0) | 12-bit, 480-cycle sampling, single conversion | Long sampling for the divider source resistance |
| PB6, PB7 | Pull-up enabled if encoder outputs are open-collector | Needed for a clean signal |

Save the project as an `.ioc` file and select it in the Simulink STM32 hardware settings.

## Simulink model

Fixed-step discrete solver, step size equal to the sample time `Ts` (about 0.1 s, the same on every block). Five groups:

| Group | Blocks | Output |
| --- | --- | --- |
| A. Encoder to speed | TIM4 Count (uint16) → Difference → Convert to int16 → Convert to double → Gain | `motor_speed` in rpm |
| B. Supply voltage | ADC1 (uint16) → Convert to double → Gain 0.003465 | `Vs` in volts |
| C. Feed-forward ratio | Constants (12 V, 2 V) → Sums → Product → Saturation 0.8 to 1.5 → Switch (`FF_on`) | `ratio` |
| D. Controller | Ref → Sum (error) → PI → +35% offset → × ratio → Saturation 0 to 100 → PWM Output (TIM1 CH1) | duty in percent |
| E. Direction | Constant 1 → GPIOA pin 9, Constant 0 → GPIOC pin 7 | IN1, IN2 |

Key settings:

- **Speed gain:** `60 / (12 × 4 × 21 × 0.098) = 0.607 rpm per count`. The sample time in the formula must match the model's sample time.
- **Counter wrap:** the Difference block has saturation off, and the result is converted to `int16`, so the 65535 to 0 wrap gives the correct signed change.
- **PI controller:** Kp = 0.03, Ki = 1, discrete forward Euler, output limited to −35 to +65, clamping anti-windup, derivative term off.
- **Deadband offset:** 35%, because the motor does not turn below about 35% duty.
- **Tunable switches:** `FF_on` (feed-forward on or off) and `sensorless_on` (encoder speed or estimated speed as feedback) are changed live during a run.

### Sensorless estimator

Speed is estimated from the motor voltage and current instead of the encoder:

```
Vm      = duty/100 * Vs - Vdrop
w_est   = (Vm - I * Ra) / Ke        (Ra = 24 ohm)
```

The estimate is first checked against the encoder, then `sensorless_on = 1` feeds it to the PI block. <!-- Edit this section to match exactly what you implemented (current shunt or voltage-only model). -->

## How to run

**Requirements**

- MATLAB R2026b with Simulink
- Embedded Coder and the Embedded Coder Support Package for STMicroelectronics STM32 Processors
- STM32CubeMX, STM32CubeProgrammer, Arm GNU toolchain
- USB cable to the Nucleo (ST-LINK)

**Steps**

1. Wire the hardware as in the pin map. Remove the ENA jumper.
2. Open the CubeMX project (`.ioc`) and confirm the settings above.
3. In MATLAB, run the STM32 hardware setup and select the Nucleo-F446RE and the `.ioc` file.
4. Open the Simulink model. Check `Ts` is the same on the TIM4, ADC and PI blocks and in the model settings.
5. Switch the **12 V supply on first**, then press **Monitor & Tune** on the Hardware tab.
6. Change `Ref`, `FF_on` and `sensorless_on` live. View signals in the Data Inspector.

**What Monitor & Tune does:** generates C code from the model, builds it with the Arm toolchain, flashes the Nucleo through ST-LINK, runs the model step on the board every `Ts`, and streams logged signals back to MATLAB. The simulation Run button only simulates on the PC and does not move the motor.

## Tasks and results

| # | Task | Method | Result |
| --- | --- | --- | --- |
| 1 | Armature resistance | Terminal resistance measurement | Ra = **24 Ω** |
| 2 | No-load speed vs voltage, Ke | Duty sweep, motor voltage by multimeter, speed by encoder and tachometer | Motor starts at about **35% duty**. Ke = _fill in_ V per rad/s |
| 3 | Starting current | Series shunt, compare with (V − Vdrop) / Ra | Theory about 0.42 A. Measured = _fill in_ |
| 4 | Speed vs load torque | Fan-blade load at fixed duty | Speed falls as load current rises |
| 5 | Efficiency | η = (T × ω) / (V × I) at several speeds | _fill in_ |
| 6 | Closed-loop PI speed control | Kp = 0.03, Ki = 1, offset 35%, clamping | Follows reference steps with quick settling |
| 7 | Step response | Reference steps of 60 → about 150 rpm and about 150 → 75 rpm | Rise time = _fill in_, overshoot = _fill in_, settling time = _fill in_ |
| 8 | Disturbance rejection | Supply 12 V → 10 V at Ref = 75 rpm; PI only vs PI with feed-forward vs sensorless | Dip and recovery = _fill in_ |

Observed with the controller: Ref 60 rpm used about 45% duty, about 75 rpm used about 56%, and about 150 rpm used about 97% duty. The 75 rpm reference was chosen for the voltage-drop test because 150 rpm leaves almost no duty headroom.

<!-- Add plots here:
![Step response](docs/images/step_response.png)
![Voltage drop test](docs/images/task8_voltage_drop.png)
-->

## Key formulas

| Topic | Formula |
| --- | --- |
| Motor voltage balance | `V = Ke * w + I * Ra` |
| Starting current | `Ist = (V - Vdrop) / Ra` |
| PWM average voltage | `Vm = D * Vs - Vdrop` |
| PWM frequency | `f = 180 MHz / ((PSC + 1) * (ARR + 1)) = 10 kHz` |
| Encoder resolution | `12 * 4 * 21 = 1008 counts per output revolution` |
| Speed from counts | `rpm = dcount / 1008 * 60 / Ts` |
| Supply voltage from ADC | `Vs = count * 3.3 / 4095 * 43 / 10 = count * 0.003465` |
| PI (discrete) | `u[k] = Kp * e[k] + Ki * Ts * sum(e)` |
| Duty command | `duty = (35 + u) * ratio`, limited to 0 to 100% |
| Feed-forward ratio | `ratio = (Vnom - Vdrop) / (Vs - Vdrop)`, limited to 0.8 to 1.5 |
| Efficiency | `eta = (T * w) / (V * I)` |

## Problems met and fixes

| Problem | Cause | Fix |
| --- | --- | --- |
| Motor would not turn with PWM applied | IN1 and IN2 both 1 (brake) | IN1 = 1, IN2 = 0 |
| Motor ran at full speed at any duty | ENA jumper cap left on | Remove the cap, wire PA8 to ENA |
| Speed stuck at maximum, reference ignored | PI fed the speed instead of the error, or positive feedback | Error = Ref − speed, feedback input negative |
| Spikes in the speed signal | 16-bit counter wrap | Difference with saturation off, then `int16` |
| Speed jumped in steps of about 6 rpm | Ts = 0.01 s gives 1 count = 5.95 rpm | Longer Ts (about 0.1 s) gives 0.6 rpm per count |
| Simulink speed disagreed with tachometer | Gain not matched to the real sample time | Gain = 60 / (12 × 4 × 21 × 0.098) |
| Large jump at the start of a run | Supply switched on after the run began, integrator wound up | Supply on first, clamping anti-windup |
| Duty could not fall below 35% | PI lower limit was 0 after adding the offset | PI limits −35 to +65 |

## Repository layout

Adjust to match your files.

```
.
├── README.md
├── simulink/            Simulink models (closed loop, Task 8)
├── cubemx/              STM32CubeMX project (.ioc)
├── scripts/             MATLAB scripts (Task 8 analysis)
├── docs/images/         Photos, wiring, plots
└── report/              Report and viva notes
```

## Safety notes

- Never connect 12 V directly to an ADC pin. Use the divider, and keep the pin voltage below 3.3 V.
- Keep the shunt voltage read by the ADC below 3.3 V. A series shunt adds resistance to the motor circuit.
- Connect the common ground before powering the motor.
- Switch the supply on before starting Monitor & Tune.

## Author

`<your name>`, `<course / institution>`

## License

Choose a licence (for example MIT) and add a `LICENSE` file.
