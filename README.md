# PulseOx — MAX30102 pulse oximeter for ATmega32A

Bare-metal AVR-C firmware. No Arduino core, no floating point, no `stdio`.
Heart rate, SpO₂, perfusion index, HRV, respiration rate and a live PPG
waveform on a 128×64 SSD1306, driven entirely by one button.

| Build | Flash | of 31.5 kB usable | SRAM | Boot |
|---|---|---|---|---|
| `release` — ship this | 28 630 B | 88.8 % | 1 752 B | ~1 s |
| `csv` — data capture | 29 930 B | 92.8 % | 1 757 B | ~1 s |
| `atmega32` — everyday, default | 30 596 B | 94.9 % | 1 766 B | ~1 s |
| `probe` — one-shot, per board | 31 342 B | 97.2 % | 1 782 B | ~6 s |

"Usable" is 32 768 B less the 512 B of urboot at the top of flash. The build
**fails** rather than silently overwriting the bootloader — see
[Flash and SRAM budget](#9-flash-and-sram-budget).

> **Not a medical device.** This is an engineering/hobby instrument. The SpO₂
> figure uses the generic Maxim empirical curve; a real oximeter is calibrated
> against arterial blood-gas measurements on human subjects. Do not use it for
> diagnosis or to make any health decision.

---

## 1. Wiring (MightyCore "standard" ATmega32 pinout)

Physical pin numbers are ATmega32 **PDIP-40**. The MightyCore column is the
logical pin number the *standard* pinout assigns, for cross-referencing against
Arduino-style wiring diagrams — this firmware does not use the core, so it
addresses ports directly.

| Function | Port bit | PDIP-40 pin | MightyCore std | Notes |
|---|---|---|---|---|
| Status LED | `PB0` | 1 | `D0` (`LED_BUILTIN`) | urboot blinks it too |
| OLED `CS` | `PB1` | 2 | `D1` | |
| OLED `DC` | `PB2` | 3 | `D2` | |
| OLED `RES` | `PB3` | 4 | `D3` | **also `OC0`** — see below |
| SPI `/SS` | `PB4` | 5 | `D4` | unused, held high by pull-up |
| OLED `D1/MOSI/SDA` | `PB5` | 6 | `D5` | hardware SPI data |
| SPI `MISO` | `PB6` | 7 | `D6` | unused, left an input |
| OLED `D0/SCK/SCL` | `PB7` | 8 | `D7` | hardware SPI clock |
| UART0 `RXD` | `PD0` | 14 | `D8` | bootloader only |
| UART0 `TXD` | `PD1` | 15 | `D9` | bootloader **and** diagnostics |
| Button → GND | `PD2` | 16 | `D10` | `INT0`, the wake source |
| Buzzer (optional) | `PD5` | 19 | `D13` | `OC1A` |
| MAX30102 `SCL` | `PC0` | 22 | `D16` (`SCL`) | hardware TWI |
| MAX30102 `SDA` | `PC1` | 23 | `D17` (`SDA`) | hardware TWI |

Sources: ATmega32A datasheet pin configuration; MightyCore
[`variants/standard/pins_arduino.h`](https://github.com/MCUdude/MightyCore/blob/master/avr/variants/standard/pins_arduino.h).

### Peripheral and interrupt map

| Resource | Used for | Notes |
|---|---|---|
| Timer0 | 1 ms system tick | CTC, ÷64, `OCR0 = 249` → exactly 1000 Hz at 16 MHz |
| Timer1 | buzzer tone | only when `USE_BUZZER 1`; otherwise idle |
| Timer2 | status-LED software PWM | CTC, ÷64; two interrupts per PWM period |
| `INT0` | wake from power-down | low-level, the only kind that works in power-down |
| TWI | MAX30102 | 100 kHz |
| SPI | SSD1306 | master, mode 0, f<sub>osc</sub>/4 = 4 MHz |
| USART0 | diagnostics (TX only) | shares `PD1` with the bootloader |
| WDT | 2 s | disabled in `.init3`, armed after the splash |

Three latent conflicts, all currently handled — worth knowing before you change
a timer:

- **`PB3` is both the OLED reset and `OC0`.** Timer0 runs in CTC for the tick
  with `COM01:COM00 = 00`, so it does not drive the pin. Enabling Timer0's
  compare output would hold the display in reset.
- **`PD7` is `OC2`.** Timer2 likewise runs with its compare output detached; the
  LED is driven by writing the port from the ISR.
- **`PB4` must not float.** It is the AVR's hardware `/SS`. It is not the chip
  select here, but if `/SS` is an input and goes low while the SPI is a master,
  the hardware clears `MSTR` and silently drops out of master mode mid-transfer
  — the display then freezes or garbles at random. `spi_init()` enables PB4's
  pull-up to prevent that. A pull-up rather than a driven output, so it cannot
  contend with anything you have wired to the pin. If something *actively*
  drives PB4 low, move that signal or make PB4 the chip select again
  (`OLED_CS` in [src/config.h](src/config.h) — the guard in
  [src/ssd1306.c](src/ssd1306.c) handles either case).

### SSD1306 128×64 OLED — hardware SPI, PORTB

| OLED pin | AVR pin | Notes |
|---|---|---|
| `VCC` | 3V3 or 5V | per your module |
| `GND` | GND | |
| `D0 / SCK / SCL` | **PB7** (SCK) | hardware SPI clock |
| `D1 / MOSI / SDA` | **PB5** (MOSI) | hardware SPI data |
| `RES` | **PB3** | |
| `DC`  | **PB2** | |
| `CS`  | **PB1** | |

The panel is driven at f<sub>osc</sub>/4 = 4 MHz, not /2. 8 MHz sits right at
the edge of the SSD1306's ~100 ns write cycle and beyond what these modules
hold reliably over jumper wires; marginal writes land as corrupted pixels,
which reads as a flickering panel. The whole 1 kB framebuffer still goes out in
about 2 ms.

Many 0.96" 128×64 modules are **two-colour**, not monochrome: the top 16 rows
of glass are yellow and rows 16–63 blue. The split is in the glass, so it lands
in the same place whatever is drawn — which is why every screen's header is
exactly 16 rows tall. The header is bright text on black rather than a filled
bar, because lighting 2 048 pixels at once loads the panel's own charge pump
hard enough to sag its supply and flicker the whole display.

### MAX30102 — hardware TWI, PORTC

| Module pin | AVR pin | Notes |
|---|---|---|
| `VIN` | 3V3 (or 5V if the module has a regulator) | |
| `GND` | GND | |
| `SCL` | **PC0** | 4.7 kΩ pull-up to the module's 3V3 rail |
| `SDA` | **PC1** | 4.7 kΩ pull-up to the module's 3V3 rail |
| `INT` | *not connected* | not required — the driver polls the FIFO |

Most GY-MAX30102 breakouts already fit the pull-ups and a 3V3 LDO. The
MAX30102 is **not** 5 V tolerant on its I²C lines — if your board lacks level
shifting, run the AVR at 3.3 V (and then use an 8 MHz crystal, or accept
out-of-spec operation at 16 MHz).

See [Electrical review](#12-electrical-review) for what is confirmed about the
breakout versus what is assumed.

### Button and status LED

| Signal | AVR pin | Notes |
|---|---|---|
| Button to GND | **PD2** | internal pull-up; also `INT0`, the wake source |
| Status LED | **PB0** | MightyCore `LED_BUILTIN`; urboot blinks it too |

No external resistor or capacitor on the button; debouncing is in software.
The button *must* stay on PD2 — on the ATmega32 only a low-level interrupt on
`INT0`/`INT1` can wake the chip from power-down, because everything else needs
the I/O clock that power-down stops.

### Optional buzzer

Set `USE_BUZZER 1` in [src/config.h](src/config.h) and wire a piezo to
**PD5** (`OC1A`). Off by default.

---

## 2. Building

### PlatformIO (canonical)

Four environments, all the same source and the same board. They exist because
32 kB of flash minus a bootloader cannot hold the everyday firmware *and* a
full verbose diagnostic stream *and* a one-shot factory probe with any working
margin.

```sh
pio run                        # builds all four
pio run -e release             # production
pio run -e release -t upload   # and flash it
pio device monitor             # 38400 8N1, diagnostics
```

| Environment | `DBG_MODE` | Status line | Channel probe | Use it for |
|---|---|---|---|---|
| `atmega32` *(default)* | 1 | yes | no | everyday work and debugging |
| `probe` | 1 | no | **yes** | once per board — see below |
| `release` | 0 | — | no | shipping |
| `csv` | 2 | no | no | capturing datasets |

**Flash `probe` once on a new board.** It establishes which FIFO word belongs
to which emitter by driving one LED at a time and watching which word follows,
then stores the verdict in EEPROM. Every other build reads it from there. Then
go back to `atmega32` or `release`. Re-run it if you swap the sensor or do a
factory reset; the [Diagnostics](#7-diagnostics) section explains the verdicts.

### Make + avr-gcc

```sh
make            # build pulseox.hex (equivalent to env:probe)
make test       # run tests/math_check.py
make flash      # avrdude, defaults to usbasp
make fuses      # 16 MHz crystal, JTAG off  (verify before writing!)
make clean
```

`make` builds the largest configuration and has **no budget check** — only the
PlatformIO build enforces the bootloader and stack reserves. Prefer `pio run`.

Fuses written by `make fuses`: `lfuse=0xFF` `hfuse=0xC9`
— external crystal ≥ 8 MHz, slow rising power, `CKOPT` programmed (full-swing
oscillator, correct for 16 MHz), JTAG disabled, `SPIEN` enabled, `BOOTRST`
**un**programmed (urboot is a vector bootloader and does not use the boot
reset vector).
**Check these against your board before writing — a wrong `lfuse` can leave the
chip unreachable without an external clock.**

### arduino-cli + urboot bootloader (no ISP programmer needed)

MightyCore's ATmega32 offers `bootloader=uart0` — that value **is** urboot
(`upload.protocol=urclock`, flashing `urboot_atmega32_pr_ee_ce.hex`). There is
no `bootloader=urboot` value; using it will fail FQBN validation.

```sh
make                       # produces pulseox.hex and pulseox.eep
arduino-cli upload -p COM3 \
  -b MightyCore:avr:32:pinout=standard,bootloader=uart0 \
  --input-file pulseox.hex
```

The recipe writes `pulseox.eep` as well as the flash, which **zeroes the saved
settings** on every upload — including the cached channel-probe verdict, so the
`probe` build has to be re-run. To keep your settings across uploads, use the
flash-only target instead:

```sh
make upload SPORT=COM3     # urclock, flash only, EEPROM untouched
make verify SPORT=COM3     # read back and compare
```

`verify` will report three "mismatches in r/o areas" at `0x0000`, `0x0001` and
`0x0052` and then pass. That is expected: urboot is a *vector* bootloader, so
it owns the reset vector (rewritten to an `rjmp` into the bootloader near
`0x7E00`) and stashes the application's original vector in a spare slot.
avrdude marks those locations read-only and ignores them.

> **If you use the bootloader, do not run `make fuses`.** Those fuses are for
> the ISP / no-bootloader path. The fuses were already set correctly when the
> bootloader was burned; rewriting them risks leaving the board unreachable.

### Arduino IDE / MightyCore

This is plain C, not a sketch, and `main()` here replaces `setup()`/`loop()`.
Use `pio run` or `make`.

---

## 3. Controls — one button, three gestures

| Gesture | On a screen | In the menu | Editing a value |
|---|---|---|---|
| **Short press** | next screen | next item | next value |
| **Double press** | previous screen | leave menu | cancel the edit |
| **Hold** (0.6 s) | open menu | select / run | confirm + save |

The gesture map is also on the **Controls** screen (menu → Controls), which
doubles as the about/health screen: firmware version, why the board last
restarted, and the measured free stack.

### Screens

1. **MONITOR** — large 7-segment BPM, SpO₂, perfusion index, live waveform strip,
   acquisition progress bar until the reading locks. When SpO₂ is unavailable it
   says so and shows *R* and the channel correlation instead of a number.
2. **WAVEFORM** — full-width PPG trace with HR/SpO₂ in the header.
3. **TRENDS** — 128 s scrolling history of HR and SpO₂, auto-scaled.
4. **ANALYSIS** — session time, beat count, HR min/avg/max, SpO₂ min, HRV
   (SDNN & RMSSD), respiration rate, sensor die temperature, quality index.
5. **SENSOR** — part/rev ID, measured sample rate, LED drive currents, DC and AC
   levels per channel, the ratio-of-ratios *R*, and the last inter-beat interval.

### Reported states

The display never shows a plausible-looking number it has not measured. What
you get instead:

| Shown | Means |
|---|---|
| `PLACE FINGER` | IR reflection has not risen over the learned idle level |
| `ACQUIRING` + progress | a pulse is being tracked, fewer than 4 accepted beats |
| `READY` | the pulse rate has converged |
| `---` in place of a value | that particular metric has no valid measurement |
| `NO SpO2  R nnn  c nn` | a pulse is tracked but *R* is outside the curve's domain, or red and IR are not moving together. The two numbers say which |
| `NO DATA FROM SENSOR` | the part answers on the bus but is not converting |
| `SENSOR FAULT` | it does not answer at all — see [Troubleshooting](#4-troubleshooting-sensor-fault) |
| quality bars, `Q nn` | signal-quality index for the current measurement |

Heart rate and SpO₂ are published **independently**. A weak red return fails
the correlation gate often and legitimately, and that must not withhold a
perfectly good pulse rate.

### Settings (persisted to EEPROM with a CRC)

LED drive (Auto/Low/Med/High) · sample averaging (1×/2×/4×) · SpO₂ trim (±5.0 %) ·
contrast · 180° flip · beat beep · auto-dim · **auto-sleep** · home screen ·
clear session · factory reset.

Averaging is 1×/2×/4× and not up to 32×, and it no longer changes the sample
rate — see [Sample rate](#51-sample-rate-and-why-averaging-does-not-change-it).

### Status LED (PB0)

PB0 has no timer output on the ATmega32, so brightness is a 256-level software
PWM. Timer2 runs in CTC and the ISR reloads `OCR2` with the remaining slice,
toggling the pin once each time — **two** interrupts per PWM period regardless
of resolution: 977 Hz refresh, ~1 950 interrupts/s, well under 1 % of the CPU.
The breathing curve squares a half-sine so the fade looks even.

| Pattern | Meaning |
|---|---|
| slow breathe | idle, waiting for a finger |
| double blip | finger present, still acquiring |
| dim + flash per beat | locked, flashing in time with your pulse |
| 2 Hz blink | sleep countdown running |
| two winks, then dark | going to sleep |
| one wink | woke up |

### Deep sleep

With **Auto Sleep** set (Off / 30 s / 1 min / 2 min / 5 min, default 2 min), the
idle timer runs whenever no finger is detected and no button is pressed. Nine
seconds before it expires a full-screen countdown appears — any press cancels it
and restarts the timer.

On expiry the firmware shuts things down in order: LED PWM, LED drive currents
zeroed, MAX30102 into register-preserving shutdown (~0.7 µA), the OLED panel
*and its charge pump* (that DC-DC is what actually draws the milliamps), SPI,
TWI, the 1 ms tick, and finally the watchdog — which **must** go last and must
go off, or it would reset the board two seconds into the sleep. The MCU then
enters power-down with a low-level `INT0` armed on the button.

Pressing the button wakes it: the ISR disables `INT0` immediately (a level
interrupt re-fires for as long as the button is held), then everything is
brought back up, the sensor is taken out of shutdown, and the measurement state
is reset. The press that wakes the board is swallowed rather than counting as a
UI gesture.

---

## 4. Troubleshooting: "SENSOR FAULT"

The fault screen is a diagnostic, not just an error. It shows the live bus
state, so you can tell these apart without a scope:

| Screen says | Meaning |
|---|---|
| `SCL L` or `SDA L` + `LINE LOW: pull-ups?` | a line is stuck low at idle — missing/absent pull-ups, a short, or a slave clamping the bus. Nothing can work until this is fixed. |
| lines `H`/`H`, `no device on the bus` | bus is electrically healthy but nothing ACKs — check `VIN`/`GND` and that SDA/SCL are not swapped. |
| `found N device(s)` with addresses listed | something is alive. If `0x57` is missing, the sensor is not powered or is a different part. |
| `0x57 ID 0xNN` | a chip answered at the MAX30102 address. `0x15` is expected and covers MAX30102 and MAX30105. **`0x11` means it is a MAX30100** — different register map, this firmware will not drive it. Any other ID is configured anyway and judged by whether the configuration reads back. |
| `0x57 did not answer` | no ACK at the sensor's address. |

The board no longer stops on this screen. After three attempts it hands over to
the main loop, which keeps retrying once a second while leaving the screens,
the menu and auto-sleep reachable — so you can read the firmware version and
the reset cause on a board whose sensor is unplugged.

The header shows the **bus speed actually in use**. The bus starts at
**100 kHz**, because these modules are routinely marginal at 400 kHz over
jumper wires and the escalation path drops to 100 kHz anyway after twenty
consecutive failures. If you want the faster bus, shorten the wires, fit
2.2 kΩ pull-ups, and change the default in `i2c_set_speed_khz()`.

If the sensor answers but nothing streams, the MONITOR screen shows
`NO DATA FROM SENSOR` with `MODE`/`SPO2`/`FIFO` read back off the chip and the
driver's own verdict:

- `CFG OK, not running` — the part is configured and simply is not converting.
  That is a **power** problem: check `VIN`, the module's LDO, and decoupling.
- `CFG BAD: not set up` — the writes are not landing. That is a **bus**
  problem; the `TW`/`E`/`S` counters on the same screen say how it is failing.

### "NO SpO2  R nnnn  c nn"

A pulse is being tracked but no saturation is published. The two numbers say
why. `R` is the ratio-of-ratios ×1000; `c` is the red/IR correlation ×100.

| `R` | `c` | Diagnosis |
|---|---|---|
| **1300–2300** | **≥ 95** | **Channels reversed.** Both channels carry a real, in-step pulse, but *R* has arrived as 1/*R*. Take the reciprocal: 1822 → 1/1.822 = 0.549 → 97.9 %, a plausible healthy reading. Fix below. |
| > 1200 | < 80 | Red is not tracking the pulse at all — dead or unseen emitter. Run `env:probe`; expect `NORED`. |
| 900–1300 | 80–100 | Common-mode interference: ambient light or movement lands on both channels equally, and red has the smaller DC to divide by, so it always pushes *R* up. Shroud the sensor, move away from windows, fluorescent lamps and IR remotes, and press a little firmer. |
| < 400 | any | Implausibly low — usually a red AC span inflated by noise on a very weak signal. Check `pi`; below 0.2 % there is not enough signal to measure. |

**Fixing a reversed pair.** Two ways:

1. **Run the probe** — `pio run -e probe -t upload`. It drives one emitter at a
   time and reports which FIFO word follows, then stores the verdict in EEPROM
   where every other build reads it. Expect `IR/RED REVERSED`. Then reflash the
   build you normally run. A finger on the sensor and dim surroundings make the
   verdict more reliable.

2. **Tell it directly**, if you already know the answer or the probe is
   inconclusive — add to your environment in [platformio.ini](platformio.ini):

   ```ini
   build_flags = ${common.build_flags} -DSENSOR_WORD_ORDER=1
   ```

   `1` forces reversed, `0` forces the datasheet order, `-1` (default) uses the
   probe's cached verdict. Setting it skips the probe entirely and does not
   depend on EEPROM surviving.

> **If you upgraded from 1.0.0 and this appeared:** that is expected on a part
> with reversed channels. 1.0.0 ran the probe on *every* boot; 1.1.0 caches the
> verdict and the default build has no probe compiled in, so a board that has
> never run `env:probe` has an empty cache and falls back to the datasheet
> order. Run `env:probe` once, or set `SENSOR_WORD_ORDER`.

Note that `pio run -t upload` is flash-only and preserves the cache, but
`arduino-cli` and `make flash` write `pulseox.eep` and **wipe it** — after those,
re-run `env:probe`.

Other things worth checking:

- **Module power.** Most GY-MAX30102 breakouts need `VIN` on 3.3–5 V and have
  their own regulator. Feeding 3V3 into the wrong pad leaves the chip unpowered
  while the board still looks connected.
- **Logic levels.** The MAX30102 is a 1.8 V part behind level shifting. If your
  board's pull-ups go to a 1.8 V rail rather than 3.3 V, a 5 V AVR will never
  see a valid high (`V_IH` = 0.6 × V_CC = 3.0 V) and the bus will look dead.
  Running the AVR at 3.3 V fixes it.
- **`PC0`/`PC1` are correct** for the ATmega32 — JTAG sits on PC2–PC5, so it
  cannot interfere with the I²C pins even if the JTAG fuse is enabled.

---

## 5. How the measurement works

### Front end

MAX30102 in SpO₂ mode (`MODE = 0x03`, red + IR): 411 µs pulse width (18-bit
ADC), 4096 nA range, 400 Hz ADC rate with 4× on-chip averaging → **100 Hz out
of the FIFO**.

400 Hz at 411 µs is the *highest rate the datasheet permits* in SpO₂ mode —
Table 11, "SpO₂ Mode (Allowed Settings)", marks 800 Hz and above as not
available at that pulse width. Longer pulse width buys ADC resolution; this is
the resolution-maximising corner of the allowed space.

Each sample is two 3-byte words. The FIFO holds 32 samples — 320 ms of buffer,
which is the stall budget every blocking operation in the firmware has to fit
inside. Up to 8 samples are read per poll into a 48-byte buffer and parsed
afterwards, rather than being handed to the DSP between reads of an open burst:
the DSP is a median sort, an integer square root and several 32-bit divides,
and running it mid-transaction held SCL low for milliseconds and left a long
window in which any disturbance aborted the transfer.

### 5.1 Sample rate, and why averaging does not change it

`SMP_AVE` averages adjacent ADC samples on-chip and pushes one result into the
FIFO, so it **divides** the FIFO output rate. Setting it alone made the
Averaging menu a hidden sample-rate control spanning a factor of 32 — and every
filter corner below is a fixed shift that is only correct at one rate.

`max30102_set_avg()` therefore writes `SPO2_SR` to compensate, holding the
output at 100 Hz:

| Averaging | `SPO2_SR` | ADC rate | FIFO out |
|---|---|---|---|
| 1× | 100 Hz | 100 Hz | 100 Hz |
| 2× | 200 Hz | 200 Hz | 100 Hz |
| 4× *(default)* | 400 Hz | 400 Hz | 100 Hz |

8× would need an 800 Hz ADC rate, which Table 11 does not allow at 411 µs — and
the part would not fail cleanly either: the datasheet says it silently programs
"the highest possible sample rate" instead, so the register would read back
400 and the output rate would halve with nothing reporting it. Hence the 4×
ceiling (`MAX_AVG_MAX`).

Averaging now means only what its name says: how much on-chip noise averaging.

**The nominal rate is not the real rate.** The MAX30102's internal oscillator is
only good to a few percent — this board measures nearer 120 Hz where 100 is
nominal — and because beats are timed by counting samples, that error lands
directly on BPM. The firmware counts actual samples against the 16 MHz crystal
over one long baseline and uses the *measured* rate, storing it in EEPROM
(stamped with the averaging setting it was measured at) so the next power-up
starts correct instead of spending the first measurement converging.

### 5.2 Signal chain (per sample, integer only)

```
raw ──▶ DC tracker (1-pole HP, 0.25 Hz) ──▶ HP2 (1-pole, 0.5 Hz)
    ──▶ LP × 2 (1-pole each, 2.4 Hz) ──▶ band-limited PPG
```

Every stage is `y += (x - y) >> k`, whose corner is `fs / (2π·2^k)`. Corners
quoted at the 100 Hz output rate.

| Stage | Shift | Corner | Purpose | Cost |
|---|---|---|---|---|
| DC tracker | `DC_SHIFT 6` | 0.25 Hz | separates DC (for the ratio and for finger detection) from AC | 1 × int32 per channel |
| HP2 | `HP2_SHIFT 5` | 0.5 Hz | removes residual baseline wander from breathing and finger movement | 1 × int32 per channel |
| LP ×2 | `LP_SHIFT 3` | 2.4 Hz | rejects noise and keeps the dicrotic notch from crossing back over zero | 2 × int32 per channel |

Latency is one time constant per stage — roughly 130 ms through the low-pass
pair, which is well inside one cardiac cycle and does not affect interval
measurement, since intervals are differences between crossings that are all
delayed equally.

2.4 Hz costs about 6 dB at a 3 Hz (180 bpm) fundamental. The zero-crossing
detector does not care about amplitude, so that is a fair trade for notch
rejection. Both channels get the **identical** filter, so the red/IR amplitude
ratio that SpO₂ depends on survives untouched.

Nothing here allocates a sample window — that is what makes it fit beside a
1 kB framebuffer on a 2 kB part.

### 5.3 Beat detection

Beats are taken from the **upward zero crossing** of the band-passed IR trace,
as the reference implementation does (SparkFun `heartRate.cpp`
`checkForBeat()`). A band-passed PPG crosses zero upward exactly once per
cardiac cycle whatever its amplitude, and the dicrotic notch sits on the
negative side without crossing back — so the detector is immune by construction
to the two things that defeat a threshold: beat-to-beat amplitude variation
from breathing, and the notch re-arming the detector and firing twice.

The crossing instant is **linearly interpolated between samples** (1/256 of a
sample), so interval resolution is ~39 µs rather than 10 ms. Without it, one
sample of jitter is 5.4 bpm at 180 bpm; with it, 0.02 bpm. That is what makes
the HRV figures meaningful.

Amplitude is used **only** as a noise gate — reject a flat trace, keep the
window wide (an eighth to eight times the running mean), and let the median
downstream deal with outliers. Intervals are accepted between 250 ms and
3000 ms (240–20 bpm), with a loose ±50 % continuity check that only engages
once eight intervals are banked, so it can catch a gross jump without ever
being able to block acquisition. BPM is the **median** of the last 12 accepted
intervals, so one dropped or extra beat cannot move it.

A refractory period of 60 % of the median interval (floor 200 ms, ceiling
700 ms) blanks the detector after each beat.

### 5.4 SpO₂

Per beat, peak-to-peak of the filtered red and IR traces gives the AC terms and
the DC trackers give the DC terms:

```
R    = (AC_red / DC_red) / (AC_ir / DC_ir)
SpO2 = −45.06·R² + 30.354·R + 94.845      (evaluated in Q8 fixed point)
```

*R* is median-filtered over the last 8 beats before the curve is applied.

Two gates decide whether a number is published at all:

- **Channel agreement.** The Pearson correlation between the two band-passed
  channels must be ≥ 0.8, which is the reference's own threshold. A cardiac
  pulse drives red and IR in lockstep and correlates better than 0.95; ambient
  light, movement, optical crosstalk or a channel not seeing its emitter all
  break that lockstep while still leaving a plausible-looking *R*. Costs three
  running sums, and the correlation comes out free alongside them.
- **Curve domain.** Past `R = 1.152` the polynomial passes 70 % and dives —
  zero at `R = 1.9`, negative beyond. An *R* out there is not a low saturation,
  it is a measurement that has gone wrong. Nothing is published and the display
  says which gate failed.

A published SpO₂ is **retired after 6 s** without a beat window yielding a
usable ratio. Beats can keep arriving normally while no ratio is obtainable, and
the last value must not sit on the display asserting a saturation nothing has
measured.

The quadratic term evaluates as one shift of 16 rather than two of 8. Truncating
twice made the fixed-point curve sawtooth: six points in the trusted range where
a *larger* ratio gave a *higher* saturation. One shift is the same work,
monotonic throughout, and halves the error against the polynomial — 1.03 vs
1.58 tenths of a percent worst case, measured by `tests/math_check.py`.

### 5.5 Other metrics

- **Perfusion index** is AC/DC on the **raw** IR signal, because that is the
  definition; *R* uses the **filtered** signals, because the filter's
  attenuation cancels in a ratio but would bias PI low.
- **Per-channel LED auto-gain** trims drive current every 250 ms to hold each DC
  level in the ADC's linear region across skin tones and contact pressure.
  Samples during a gain step are discarded and the beat reference is dropped, so
  no interval is timed across the blanking window. Finger detection compares
  drive-**normalised** reflection, so it stays valid at any gain.
- **Respiration (RIIV)** band-passes the IR baseline rather than sampling beat
  amplitudes — 100 Hz instead of once per beat, which is barely above Nyquist
  for breathing. Crossings are counted over a 3 000-sample window and the
  measured rate converts it to breaths/min. **Experimental**: it is derived from
  a modulation an order of magnitude smaller than the cardiac signal, it is
  destroyed by movement or breath-holding, and it reports nothing rather than
  guessing when the modulation is below a floor. Not a clinical measurement.
- **Signal quality index** penalises low perfusion, high interval variability
  and rejected crossings. Rejections are counted **per measurement**, not per
  session, so a fresh finger placement does not inherit the previous one's
  reject ratio.

### 5.6 Accuracy — what has and has not been verified

> The table below characterises the **algorithm against synthetic signals**, not
> the optics, and it was produced for an earlier revision of the beat detector.
> Nothing in this repository has been measured against a reference oximeter on
> hardware. Treat it as a regression baseline, not an accuracy claim.

| Test | Result | Status |
|---|---|---|
| SpO₂ fixed point vs the polynomial, R ≤ 1.152 | max error **0.10 %** | verified by `tests/math_check.py` |
| SpO₂ curve monotonic over the trusted range | pass | verified by `tests/math_check.py` |
| Register configuration vs datasheet Tables 3–11 | pass | verified by `tests/math_check.py` |
| FIFO overflow time gate vs FIFO depth and poll rate | pass | verified by `tests/math_check.py` |
| `millis()` and sample-counter wraparound | pass | verified by `tests/math_check.py` |
| Heart rate, clean synthetic PPG, 45–180 bpm | < 4 % | **written, not executed here** — `tests/host/test_ppg.c`, needs a host C compiler |
| SpO₂ end to end vs constructed *R* | < 2.5 points | **written, not executed here** |
| Heart rate, clean, 35–200 bpm | max error 0.4 bpm | historical, earlier detector revision |
| SpO₂ vs the Maxim curve, 74–100 % | max error 0.19 % | historical |
| HR under motion/breathing/noise (36 runs) | mean 1.1 bpm, worst 5.1 | historical |
| SpO₂ under the same stress | mean 0.60 %, worst 2.11 | historical |
| Low-perfusion floor | works to PI 0.2 % | historical |
| Anything involving real optics, contact or ambient light | — | **requires hardware** |

### 5.7 Calibrating SpO₂

The `−45.06R² + 30.354R + 94.845` curve is Maxim's generic reference. To trim it
for your hardware, compare against a reference oximeter on a healthy subject at
rest (where true SpO₂ is 96–99 %) and adjust **Settings → SpO₂ Trim** (±5.0 % in
0.5 % steps, stored in EEPROM).

This is a single-point offset. It cannot correct the curve's shape at low
saturation, and you cannot safely generate low-saturation reference points at
home. **Do not calibrate from one person or one measurement** — a single subject
at one contact pressure tells you about that subject and that pressure. Take
readings from several people across a range of skin tones and finger sizes, at
rest, and only shift the trim if the *median* disagreement is consistent.

---

## 6. Robustness

- 2 s watchdog, cleared once per main-loop pass. `MCUCSR` is latched into
  `sys_mcucsr` (`.noinit`) and the WDT disabled in `.init3`, before anything can
  disturb either — the reset flags are sticky, so left alone the next reset
  would report this one's cause on top of its own.
- **Watchdog/bootloader interaction.** urboot is a vector bootloader and hands
  over with the WDT in whatever state it left it; `.init3` disables it
  unconditionally, so a watchdog reset cannot loop. The WDT is armed only after
  the splash, and every long-running startup step (`soft_reset`, the channel
  probe, the fault screen, the I²C scan) resets it as it goes.
- Every TWI wait is bounded — no infinite spin on a stuck bus. A NACK is exited
  by clearing the TWI rather than by executing a STOP, because from the NACK
  states this part never completes one; a pending `TWSTO` would otherwise make
  every later transaction time out at the START stage until power-cycled.
- If the sensor stops responding, the UI shows a fault screen and the main loop
  runs bus recovery (9 clock pulses + STOP) and re-initialises once a second.
  A burst that dies part way through a sample realigns the FIFO pointers, and
  reports the discarded samples so the time base stays honest.
- FIFO loss is read from `OVF_COUNTER` and **cross-checked against elapsed
  time** before it is believed — see [Diagnostics](#7-diagnostics).
- Settings are validated by magic + version + CRC-8; a bad or older-layout
  EEPROM record falls back to defaults instead of loading garbage. Writes use
  `eeprom_update_block`, so unchanged bytes are not rewritten.
- The main loop never blocks: FIFO drain, DSP, housekeeping and a 20 fps redraw,
  with at most 8 samples consumed per pass so the button stays responsive.
- All timing is unsigned-difference arithmetic, so `millis()` wrapping at 49.7
  days is a non-event. Crossing timestamps wrap every 2^24 samples (46.6 hours
  at 100 Hz) and the interval, being a difference, stays correct across it.

### Timing budget

| Item | Cost | Of the 320 ms FIFO budget |
|---|---|---|
| FIFO pointer read (one 3-byte burst, per 4 ms poll) | 470 µs | 11.8 % of wall clock |
| — as two single-register reads, before | 760 µs | 19.0 % |
| Framebuffer flush at 4 MHz SPI | 2.0 ms | 0.6 % |
| Diagnostic status line at 38400 (every 2 s) | 65 ms | 20 %, 6.5 samples |
| Timer2 LED PWM ISR | ~1 950/s | < 1 % CPU |

Figures from `tests/math_check.py`, which recomputes them from the source
constants.

---

## 7. Diagnostics

UART0 TX on **PD1**, 38400 8N1 — the same pin the bootloader uses, so anything
that can flash the board can already read this.

### Status line (`DBG_MODE 1`, one line every 2 s)

`name=value` pairs. The fields worth knowing:

| Field | Means | Healthy |
|---|---|---|
| `up` | sensor present | 1 |
| `id` | `PART_ID` | 15 |
| `mode` `spo2` `fifo` | config read back off the chip | 03, 2F at 4× averaging, 50 |
| `sps` | raw samples received per second | ~120 |
| `fs` | calibrated sample rate, Hz | ~120 |
| `lps` | main-loop passes per second | > 150 |
| `ir` `red` | DC levels | 60 000–200 000 with a finger |
| `refl` `base` `th` | drive-normalised reflection, learned idle, threshold | `refl` > `th` ⇒ finger |
| `led` `ledr` | IR and red drive codes | ×0.2 mA |
| `r` | ratio-of-ratios ×1000 | 400–800 |
| `aci` `acr` | band-passed AC spans forming *R* | |
| `corr` | red/IR Pearson correlation ×100 | ≥ 95 |
| `rail` | 0 published · 1 *R* out of domain · 2 correlation gate | 0 |
| `swap` | channel order in use, from the probe | 0 |
| `err` `stk` `tw` `stg` `ln` | I²C error count, bus force-frees, last status, stage, line levels | 0, 0, —, —, 3 |
| `ovf` | `OVF_COUNTER` as read, before cross-checking | 0 |
| `stack` | **free stack in bytes, measured** | > 80, and stable |
| `rst` | reset cause: `P`ower-on `E`xternal `B`rown-out `W`atchdog | `P` or `E` |

Two of these deserve attention on a new board:

- **`stack`** should settle in the first minute and then never move. A figure
  that keeps falling over hours means something is leaking into the stack; `0`
  means the stack has already reached `.bss` and the corruption has happened.
- **`ovf`** should be 0 except just after a real overflow. The datasheet says
  `OVF_COUNTER` resets to zero whenever a sample is popped, which this driver
  does on nearly every poll. An earlier bench observation on this hardware was
  that it read a saturated 31 regardless, so the value is cross-checked before
  it is believed — against **elapsed time**, not against the pointers. With
  rollover enabled the write pointer wraps straight past the read pointer, so
  after a real overflow the difference between them is small for a small loss
  and arbitrary for a large one; a "does the FIFO look full" test would reject
  exactly the genuine overflows it was meant to confirm. Time is the honest
  discriminator: the FIFO holds 320 ms, so a loss reported less than 200 ms
  after the last successful drain cannot be describing this bus. If `ovf` sits
  at 31 while `sps` is healthy, this board is the misbehaving kind and the gate
  is what is protecting the time base.

### Per-beat line

`B ibi=<ms> amp=<counts> r=<code>` for every crossing, accepted or not. The
code distinguishes the three failure shapes that all look like "jumping
numbers" on the display:

| Code | Meaning |
|---|---|
| `O` | accepted |
| `S` | interval too short — firing twice per beat gives alternating `S`/`O` |
| `L` | interval too long — missed beats give intervals at multiples of the true one |
| `C` | failed the continuity check against the median |
| `W` | amplitude below the noise gate |

### Channel probe (`env:probe`)

Drives one emitter at a time and reports which FIFO word follows it, because a
part that returns them the other way round inverts every *R* into 1/*R* — which
reads on the display as an SpO₂ that never moves, and which no amount of
staring at the DSP will find, because the DSP is correct and the labels on its
inputs are not.

| Verdict | Meaning |
|---|---|
| `OK` | word0 = red, word1 = IR, as the datasheet specifies |
| `REVERSED->swapped` | reversed, and corrected in the driver for the rest of the run |
| `NORED` / `NOIR` | that emitter moved neither word — dead or undriven |
| `BOTH` | no optical separation. Retry with a finger on, out of bright light |

Only `OK` and `REVERSED` are cached — the three inconclusive verdicts leave the
cache alone so the next boot tries again. The verdict and the part ID it was
established against are stored in EEPROM, so a conclusive answer is reached once
per sensor rather than on every boot. A factory reset or a different part
re-arms it, and `SENSOR_WORD_ORDER` overrides the whole mechanism.

---

## 8. Capturing data

`env:csv` emits **one CSV record per crossing** on the same UART, with a column
header at start-up. Import it straight into a spreadsheet.

```sh
pio run -e csv -t upload
pio device monitor > capture.csv     # Ctrl-C to stop
```

Columns: `t_ms, code, ibi_ms, bpm_inst_x10, bpm_x10, amp, spo2_x10, r_x1000,
pi_x100, corr, sqi, fgr, valid, rail, dc_ir, dc_red, base_ir, ac_ir, ac_red,
fac_ir, fac_red, sps, fs_x100, beats, rej, ovf, i2c_err, stuck, stack_free`.

`bpm_inst_x10` is the rate this one interval implies; `bpm_x10` is the
median-filtered figure the display shows. The gap between them is exactly what
the outlier rejection is doing, and it is not visible any other way. Rejected
crossings are included with their reason code, because the datasets worth
capturing are the ones where beats are *not* being accepted.

**Why per-crossing and not per-sample.** A per-sample stream of these fields is
about 7 kB/s at ~120 Hz, which does not fit in 38400 (3.8 kB/s) — and raising
the baud rate to carry it would leave the main loop blocked inside the
transmitter most of the time, perturbing the very timing the capture exists to
measure. Two records a second use under 10 % of the link with no measurable
effect on FIFO servicing. If you need raw samples, add a `c_u32` call per
sample in `ppg_process`, raise `DBG_BAUD` to 500000, and accept that display
timing and possibly sample servicing will suffer — capture only, never a
build to measure timing with.

### Datasets worth capturing

Capture at least 60 s of each, and note the conditions in the filename:

| Condition | What it should show | What a fault looks like |
|---|---|---|
| No finger | `fgr=0`, `refl` below `th`, no beats | beats accepted on nothing ⇒ noise gate too low |
| Stable finger, firm contact | `corr` > 95, `rail=0`, `rej` near 0 | high `rej` ⇒ detector or refractory problem |
| Weak / light contact | low `pi`, `rail=2` sometimes | a confident SpO₂ ⇒ gates too loose |
| Deliberate movement | `rej` rises, `corr` falls, SpO₂ withheld | SpO₂ still published ⇒ correlation gate not working |
| Cold finger / low perfusion | `pi` < 0.3, longer acquisition | nothing at all ⇒ noise gate too high |
| Bright ambient light | `r` rises, `corr` falls | SpO₂ drifting down without warning |
| Different finger pressure | `dc_ir` moves, `led` compensates | `dc_ir` railing ⇒ AGC window wrong |
| Unplug and reconnect the sensor | `up` 0 → 1, recovery within ~1 s | needs a power cycle ⇒ recovery path broken |

Compare against a **trustworthy reference oximeter**, and never trim the
calibration from a single person or a single measurement — see
[Calibrating SpO₂](#57-calibrating-spo2).

---

## 9. Flash and SRAM budget

The ATmega32A has 32 kB of flash and urboot lives at the top of it. `avr-size`
compares against the whole 32 kB and so never warns — so the most likely way to
break this board is not a bug, it is a build that quietly grows over the
bootloader, after which only an ISP programmer recovers it.

[`scripts/check_size.py`](scripts/check_size.py) runs after every PlatformIO
link and **fails the build** against two budgets:

| Budget | Default | Why |
|---|---|---|
| `bootloader_reserve` | 512 B | urboot's vector jump lands near `0x7E00`, so the top 512 B are its |
| `stack_reserve` | 224 B | measured worst-case call chain is 182 B |

Both are set in [platformio.ini](platformio.ini) and can be overridden:

```ini
board_upload.bootloader_reserve = 1024
```

**Verify the reserve against your own bootloader** before trusting it. Read the
top of flash with an ISP programmer, or check the size of the
`urboot_atmega32_pr_ee_ce.hex` MightyCore flashed. 512 B is the usual footprint
for this part and matches the `0x7E00` vector this project's own upload notes
record, but it is an assumption, not a measurement made here.

If a build overruns, the two biggest levers are `DBG_MODE=0` (~3.8 kB) and
`DBG_LED_PROBE=0` (~1.1 kB) — which is exactly what `env:release` does.

### Stack

The worst-case chain, measured with `-fstack-usage`:

```
main 12  →  max30102_read 62  →  ppg_process 46  →  median_u16 31
= 151 B of frames + 8 B of return addresses = 159 B
+ one interrupt frame (TIMER0_COMP, 21 B + 2) = 182 B
```

AVR interrupts do not nest, so only one interrupt frame can be present. Against
286 B of free SRAM in the default build that is a 36 % margin.

Static analysis cannot see what the optimiser did under LTO, so the firmware
**measures it**: the SRAM between `.bss` and `RAMEND` is painted with `0xC5` in
`.init1` and `sys_stack_free()` counts how much paint still stands. That figure
is on the diagnostic line as `stack` and on the Controls screen as `STK`.

`.init1` is deliberate and load-bearing: it runs before `__do_copy_data` and
`__do_clear_bss`, so neither can undo the paint. It also runs *before* `.init2`,
which is where avr-libc loads the stack pointer — SP reads 0 that early, so the
upper bound comes from the linker's `__stack` symbol and not from SP.

---

## 10. Layout

| File | Purpose |
|---|---|
| [src/main.c](src/main.c) | startup, watchdog, main loop, sensor recovery, channel-order cache |
| [src/config.h](src/config.h) | pin map and build-time options |
| [src/sys.c](src/sys.c) | 1 ms tick, `millis()`, button gestures, reset cause, stack high-water mark |
| [src/i2c.c](src/i2c.c) | TWI master with timeouts + bus recovery |
| [src/max30102.c](src/max30102.c) | sensor driver, FIFO burst reader, overflow accounting, die temp |
| [src/ppg.c](src/ppg.c) | the signal chain — filters, beat detection, SpO₂, HRV, respiration, AGC |
| [src/ssd1306.c](src/ssd1306.c) | hardware SPI + panel driver |
| [src/gfx.c](src/gfx.c) | framebuffer primitives, 5×7 text, scalable 7-segment digits |
| [src/ui.c](src/ui.c) | screens, settings menu, single-button interaction |
| [src/settings.c](src/settings.c) | EEPROM persistence with CRC |
| [src/dbg.c](src/dbg.c) | UART diagnostics: status line, per-beat line, CSV, channel probe |
| [src/buzzer.c](src/buzzer.c) | optional beeper (compile-time gated) |
| [src/led.c](src/led.c) | status LED patterns, 256-level software PWM on Timer2 |
| [src/power.c](src/power.c) | deep sleep sequencing and wake-on-button |
| [scripts/check_size.py](scripts/check_size.py) | post-link flash/SRAM budget guard |
| [tests/math_check.py](tests/math_check.py) | integer maths, registers, timing — Python only |
| [tests/host/test_ppg.c](tests/host/test_ppg.c) | the real DSP against synthetic PPG — needs a host C compiler |

---

## 11. Testing

### `tests/math_check.py` — no compiler needed

```sh
python tests/math_check.py        # 101 checks
```

Checks the SpO₂ polynomial against the float reference, fixed-point overflow
limits, beat-detector bounds, the MAX30102 register configuration against
datasheet Tables 3–11, the sampling and timing budget, `millis()` and
sample-counter wraparound, and the settings record layout.

It **parses the constants out of the source** rather than restating them, so
editing a `#define` moves the check with it and a real disagreement is what
fails — rather than the check quietly testing a value the firmware no longer
uses.

### `tests/host/test_ppg.c` — needs a host C compiler

```sh
powershell -ExecutionPolicy Bypass -File tests\host\run.ps1
# or, with gcc/clang:
cc -I tests/host/stub -I src -o test_ppg tests/host/test_ppg.c -lm && ./test_ppg
```

`#include`s the real `src/ppg.c` and `src/sys.c` against stub AVR headers, so
what runs is the firmware's own code and not a copy. It drives the DSP with a
synthetic PPG — systolic Gaussian plus a dicrotic notch at 35 % height, which is
what a threshold detector fires twice on — and covers rate accuracy 45–180 bpm,
SpO₂ against a constructed *R*, perfusion index, no-finger, finger removal,
heart rate published without SpO₂, SpO₂ staleness, `millis()` and
sample-counter wraparound, lost-sample accounting, saturation and all-zero
inputs, and the waveform ring.

> **Not executed in this repository's history.** It type-checks clean under
> `avr-gcc -Wall -Wextra`, but the machine it was written on had no host C
> compiler. Run it before relying on its results.

### What still requires hardware

None of the above touches optics, I²C timing on the real part, LED current,
power integrity or display behaviour. See
[On-device validation](#13-on-device-validation).

---

## 12. Electrical review

Separated by how much is actually known.

**Confirmed from the code and this repository:**

- The AVR runs at 16 MHz from an external crystal with `CKOPT` programmed, and
  every derived rate — the 1 ms tick, 38400 baud with `U2X`, TWI at 100 kHz,
  SPI at 4 MHz, both PWM timers — is computed from `F_CPU` and matches it.
- SDA/SCL are on the ATmega32's hardware TWI pins. JTAG (PC2–PC5) is disabled by
  `hfuse`, and could not have interfered with PC0/PC1 anyway.
- The MAX30102's `INT` pin is not used; the driver polls, as the reference does.
- Both LED drive currents are zeroed before sleep, in addition to the shutdown
  bit, so the emitters go dark even if the mode write does not land.

**Likely, given what these modules normally are:**

- A GY-MAX30102 breakout with its own 3V3 LDO, 4.7 kΩ pull-ups to that rail, and
  a 1 µF bypass — the datasheet asks for at least 1 µF as close to the IC as
  possible, and for the resistance and inductance from the supply to `VLED+` to
  be well under 1 Ω, because the LEDs are pulsed and the ripple becomes optical
  noise if the supply cannot absorb it.
- The LED ceiling is capped at `0x3F` ≈ 12.6 mA per emitter rather than the
  part's 51 mA maximum, and both emitters pulse. On a small on-board LDO, higher
  drive browns the module out — which presents as bursts of I²C failures and an
  intermittent `SENSOR FAULT`, not as anything obviously optical.

**Needs a photo or a measurement — not assumed here:**

- The breakout's actual schematic: regulator part, pull-up values and which rail
  they go to, whether level shifting is fitted, and what decoupling is present.
- Whether the AVR runs at 5 V or 3.3 V. At 5 V, `V_IH` is 3.0 V, so pull-ups to a
  1.8 V rail will never present a valid high and the bus will look dead.
- Cable length between the AVR and the sensor. Long dupont leads plus weak
  pull-ups are the usual reason 400 kHz fails; that is why the bus starts at
  100 kHz.
- Optical isolation between the emitters and the photodiode, and ambient-light
  ingress. The finger-detect threshold is learned per board precisely because
  idle reflection varies by an order of magnitude with how well the plastic
  isolates them.
- Thermal rise at the fingertip. Datasheet Table 13 gives roughly 2 °C at 50 mA
  and 8 % duty; at 12.6 mA and this duty it should be well under 1 °C, but that
  is a calculation, not a measurement.

No pin assignment has been changed, and no wiring change is recommended.

---

## 13. On-device validation

In order. Steps 1–4 need no finger.

1. **Flash `probe` once.** `pio run -e probe -t upload`, watch the UART at
   38400. Expect three `P` lines and `P order=OK`. Anything else — read the
   verdict table in [Diagnostics](#7-diagnostics) and fix it before going
   further; a reversed pair makes every SpO₂ reading meaningless.
2. **Flash the everyday build.** `pio run -t upload`. Confirm on the status
   line: `up=1`, `id=15`, `mode=03`, `spo2=2F`, `fifo=50`, `ln=3`, `err=0`,
   `stk=0`, `rst=P` or `E`, `swap=0` (or 1 if the probe said reversed).
3. **Sample rate.** With no finger, confirm `sps` is 100–130 and steady, and
   that `fs` converges to the same figure within ~5 s and then stops moving. A
   drifting `fs` means the estimator is being restarted — look for `ovf`.
4. **Stack.** Leave it running 10 minutes, cycling through all five screens and
   in and out of the menu. `stack` must settle and then hold. Note the value;
   anything under 80 B wants investigating before you ship.
5. **Finger detection.** Place a finger. `fgr` must go 0 → 1 within ~0.5 s, and
   `refl` must clear `th` by a comfortable margin. Lift it: `fgr` must return to
   0 within ~1 s. Repeat ten times — a detection that latches but will not
   release is the classic failure and it is visible right here.
6. **Rate accuracy.** Against a reference oximeter, at rest, on several people.
   Record both readings once `READY` appears and stays for 30 s. Expect
   agreement within a few bpm; a rate that reads roughly double or half means
   the detector is double-firing or missing beats, which the per-beat `S`/`L`
   codes will show.
7. **SpO₂ agreement.** Same subjects, same time. Only adjust **SpO₂ Trim** if
   the *median* disagreement across subjects is consistent. Record `r` and
   `corr` alongside; `corr` below 95 at rest means contact or optics, not
   calibration.
8. **Capture the datasets** in [Capturing data](#8-capturing-data), all eight
   conditions, and check each against the "what a fault looks like" column.
9. **Fault recovery.** Unplug the sensor's SDA while running: expect
   `SENSOR FAULT` within a couple of seconds, the screens and menu still
   reachable, and full recovery within ~1 s of reconnecting — with no power
   cycle. Then unplug `VIN` and repeat.
10. **Watchdog.** There is no deliberate hang to trigger it, so verify it
    indirectly: confirm `rst=P`/`E` in normal operation over a long run. A `W`
    appearing means something blocked for over 2 s and wants finding.
11. **Sleep and wake.** Set Auto Sleep to 30 s, leave it alone, confirm the
    countdown, the two winks, a dark panel, and that a press brings everything
    back including the sensor. Measure the sleep current if you can — the OLED
    charge pump is the thing that dominates it.
12. **Long run.** 24 hours untouched. Check `stack` has not moved, `err`/`stk`
    are still low, `rst` is unchanged, and the display has not drifted. This is
    also the run that would expose a `millis()` problem, though the wrap itself
    is 49.7 days away.

---

## 14. Tuning

Everything worth touching is a `#define` at the top of
[src/ppg.c](src/ppg.c). Change one thing at a time and capture a dataset before
and after — see [Capturing data](#8-capturing-data).

| Symbol | Effect |
|---|---|
| `DC_SHIFT` | DC-tracker corner (6 ≈ 0.25 Hz). Lower follows contact changes faster and bleeds more pulse into DC |
| `HP2_SHIFT` | wander rejection vs. low-HR response (5 ≈ 0.5 Hz) |
| `LP_SHIFT` | noise rejection vs. notch rejection (3 ≈ 2.4 Hz). Raising it lets the dicrotic notch cross zero and double the rate |
| `ENV_DECAY` | how fast the display normalisation follows changing pulse height |
| `AMP_MIN_BEAT` | noise floor — raise it for false beats, lower it for weak pulses |
| `LOCK_BEATS` | beats required before a reading is published |
| `SPO2_STALE_MS` | how long a published SpO₂ survives without a fresh ratio |
| `R_TRUST_MAX` | top of the SpO₂ curve's domain (295 ⇒ R = 1.152 ⇒ 70 %) |
| `CORR_MIN_N`/`CORR_MIN_D` | the channel-agreement gate, as a fraction (4/5 = the reference's 0.8) |
| `FINGER_MIN_RISE`, `FINGER_FLOOR_IR`, `FINGER_CAP_IR` | finger detection. `FINGER_CAP_IR` is a **cap**, not a floor — see the long note in the source |
| `RESP_WIN` | respiration window (3 000 samples; the first reading takes that long) |
| `LED_PA_MAX` in [src/max30102.h](src/max30102.h) | AGC ceiling. Raising it past 12.6 mA browns out most breakouts |
| `FIFO_POLL_MS` in [src/main.c](src/main.c) | FIFO poll interval vs. I²C bus load |
| `SLEEP_COUNTDOWN_S` in [src/ui.c](src/ui.c) | how long the countdown runs before sleeping |

---

## 15. Known limitations

- **The SpO₂ curve is uncalibrated** for any specific module, and the trim is a
  single-point offset that cannot correct its shape at low saturation.
- **Respiration is experimental** and reports nothing more often than it reports
  a number.
- The **`probe` build has 910 B of flash headroom** and the default build
  1 660 B. There is not room for both the channel probe and the verbose status
  line, which is why they are separate environments.
- **`OVF_COUNTER` behaviour on this hardware is unresolved** — the datasheet and
  an earlier bench observation disagree. The cross-check handles both, and the
  `ovf` field reports which world this board is in, but it has not been
  confirmed on the device.
- **No burn-in mitigation.** Auto-dim and auto-sleep are the mitigation. Pixel
  shifting is not implemented because the usual trick — the SSD1306's vertical
  display offset (`0xD3`) — would slide the header off the panel's yellow band
  and wrap it to the bottom, which is worse than the problem. A horizontal
  shift would need an offset threaded through every drawing call.
- **The bootloader reserve is an assumption** (512 B), consistent with this
  project's own record of the vector landing near `0x7E00` but not measured
  here.
- `tests/host/test_ppg.c` has never been executed.
- Nothing has been validated against a reference oximeter on hardware.

---

## 16. Sources

- MAX30102 datasheet, Analog Devices/Maxim — register map, Table 3 (sample
  averaging), Table 5 (ADC range), Table 6 (sample rate), Table 7 (pulse width),
  Table 8 (LED current), Table 9 (multi-LED slots), Table 11 (**allowed SpO₂
  sample-rate/pulse-width combinations**), Table 13 (LED temperature rise), FIFO
  and overflow-counter semantics, power-up sequencing.
  <https://www.analog.com/media/en/technical-documentation/data-sheets/max30102.pdf>
- SparkFun MAX3010x library — the de-facto reference driver; register masks,
  configuration order and `checkForBeat()`'s zero-crossing detector.
  <https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library>
- Maxim `algorithm_by_RF` (RD117) — the RMS-based *R* and the 0.8 Pearson
  correlation gate this firmware adopts.
- MightyCore — ATmega32 support, standard pinout, urboot as `bootloader=uart0`.
  <https://github.com/MCUdude/MightyCore>
  · pin mapping verified against
  [`variants/standard/pins_arduino.h`](https://github.com/MCUdude/MightyCore/blob/master/avr/variants/standard/pins_arduino.h)
- urboot — vector bootloader and the `urclock` avrdude programmer.
  <https://github.com/stefanrueger/urboot>
- ATmega32A datasheet, Microchip — pinout, TWI, timers, `MCUCSR` reset flags,
  watchdog, power-down and `INT0` wake.
- PlatformIO — `extra_scripts`, `AddPostAction`, environment inheritance.
  <https://docs.platformio.org/>

---

## 17. Changelog

### 1.1.0

**Correctness**

- Sample averaging no longer changes the sample rate. `SPO2_SR` is written to
  compensate `SMP_AVE`, holding the FIFO output at 100 Hz, and the range is
  limited to 1×/2×/4× because 8× would need an ADC rate datasheet Table 11 does
  not allow at 411 µs. Previously the Averaging menu moved every DSP filter
  corner by up to 32× and could stop beat detection entirely.
- Heart rate is published independently of SpO₂. It previously required a
  non-zero SpO₂, so a weak red return — a normal, expected outcome — withheld a
  perfectly good pulse rate, leaving the header stuck on `ACQUIRING` and the HR
  trend empty.
- A published SpO₂ is retired after 6 s without a fresh ratio. It previously
  persisted indefinitely while beats kept arriving.
- The SpO₂ quadratic term folds its two shifts into one: monotonic across the
  trusted range (six inversions removed) and half the error against the
  polynomial.
- The signal-quality index counts rejections per measurement, not per session,
  so a fresh finger placement no longer inherits the previous one's ratio.
- The stored sample-rate calibration is stamped with the averaging setting it
  was measured at, and discarded if it no longer applies.
- Quality index is cleared when a reading is dropped, instead of showing the
  quality of a measurement that has stopped.
- The MONITOR screen's register read-back shows the driver's live verdict
  instead of hard-coded expected values, which stopped being correct once
  `SPO2_CFG` began tracking the averaging setting.
- Factory reset restarts the rate estimator if it changed the averaging.

**Reliability**

- FIFO pointers and `OVF_COUNTER` are read in **one** 3-byte burst instead of
  two single-register transactions — 40 % less bus time on the hottest path, per
  the datasheet's guarantee that the register pointer auto-increments.
- FIFO loss is taken from `OVF_COUNTER`, gated on enough time having passed
  un-drained for the FIFO to fill, and an unquantified overflow now reports a
  small marker rather than a made-up 31 — which used to inflate both the sample
  time base and the rate calibration.
- Startup no longer blocks forever on a missing sensor. After three attempts it
  hands over to the main loop, which retries every second while leaving the
  screens, the menu and auto-sleep reachable.
- The channel-probe verdict is cached in EEPROM and applied unconditionally at
  startup — including in builds with no probe compiled in, which previously
  dropped a correction already established on that board. Only **conclusive**
  verdicts are cached: caching a `NORED`/`NOIR`/`BOTH` result made the probe
  never run again while leaving the datasheet order in place, so one
  inconclusive probe could lock in the wrong channel order permanently.
- `SENSOR_WORD_ORDER` forces the channel order from the build, for when the
  answer is already known or the probe cannot settle it.

**Production readiness**

- `scripts/check_size.py` fails the build if code+data would reach the
  bootloader, or if static SRAM leaves too little for the stack. `avr-size` warns
  about neither.
- Four build environments: `atmega32`, `probe`, `release`, `csv`.
- Reset-cause reporting (`MCUCSR` latched in `.init3`) on the UART and the
  Controls screen. The register was already being captured and never read.
- Measured stack high-water mark, painted in `.init1`, reported as `stack` and
  `STK`.
- CSV diagnostic mode: one record per crossing, 29 fields, with a header line.
- Additional warnings enabled (`-Wshadow -Wpointer-arith -Wcast-align
  -Wstrict-prototypes -Wmissing-prototypes -Wredundant-decls -Wundef`); the tree
  builds clean under all of them.
- `tests/math_check.py` (101 checks, Python only) and `tests/host/test_ppg.c`.

**Resources** — default build: flash 31 602 → 30 596 B (−1 006), SRAM
1 771 → 1 766 B. `release` is 28 630 B. Warnings: 0 before, 0 after.

**Documentation** — the README described the previous threshold-based beat
detector, a 5 Hz low-pass, a 400 kHz default bus speed, 32-level LED PWM and
symbols that no longer exist. All corrected, and the simulation accuracy figures
are now labelled with what actually verified them.

### 1.0.0

Initial release.
