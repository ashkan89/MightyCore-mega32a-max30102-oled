# PulseOx — MAX30102 pulse oximeter for ATmega32A

Bare-metal AVR-C firmware. No Arduino core, no floating point, no `stdio`.
Heart rate, SpO₂, perfusion index, HRV, respiration rate and a live PPG
waveform on a 128×64 SSD1306, driven entirely by one button.

```
Flash 21.7 kB / 32 kB (66 %)     RAM 1621 B / 2048 B (79 %)     EEPROM 11 B
```

> **Not a medical device.** This is an engineering/hobby instrument. The SpO₂
> figure uses the generic Maxim empirical curve; a real oximeter is calibrated
> against arterial blood-gas measurements on human subjects. Do not use it for
> diagnosis or to make any health decision.

---

## 1. Wiring (MightyCore "standard" ATmega32 pinout)

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

`PB6` (MISO) is left as an input and unused.

> **PB4 must not float.** PB4 is the AVR's hardware `/SS`. It is not the chip
> select here, but if `/SS` is an input and goes low while the SPI is a master,
> the hardware clears `MSTR` and silently drops out of master mode mid-transfer
> — the display then freezes or garbles at random. `spi_init()` enables PB4's
> pull-up to prevent that. A pull-up rather than a driven output, so it cannot
> contend with anything you have wired to the pin. If something *actively*
> drives PB4 low, move that signal or make PB4 the chip select again
> (`OLED_CS` in [src/config.h](src/config.h) — the guard in
> [src/ssd1306.c](src/ssd1306.c) handles either case).

### MAX30102 — hardware TWI, PORTC

| Module pin | AVR pin | Notes |
|---|---|---|
| `VIN` | 3V3 (or 5V if the module has a regulator) | |
| `GND` | GND | |
| `SCL` | **PC0** | 4.7 kΩ pull-up to the module's 3V3 rail |
| `SDA` | **PC1** | 4.7 kΩ pull-up to the module's 3V3 rail |
| `INT` | PD3 *(optional)* | not required — the driver polls the FIFO |

Most GY-MAX30102 breakouts already fit the pull-ups and a 3V3 LDO. The
MAX30102 is **not** 5 V tolerant on its I²C lines — if your board lacks level
shifting, run the AVR at 3.3 V (and then use a 8 MHz crystal, or accept
out-of-spec operation at 16 MHz).

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
**PD5** (OC1A). Off by default.

---

## 2. Building

### Make + avr-gcc

```sh
make            # build pulseox.hex
make flash      # avrdude, defaults to usbasp
make fuses      # 16 MHz crystal, JTAG off  (verify before writing!)
make clean
```

Fuses written by `make fuses`: `lfuse=0xFF` `hfuse=0xC9`
— external crystal ≥ 8 MHz, slow rising power, `CKOPT` programmed (full-swing
oscillator, correct for 16 MHz), JTAG disabled, SPIEN enabled.
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
settings** on every upload (harmless — the CRC check falls back to defaults).
To keep your settings across uploads, use the flash-only target instead:

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

### PlatformIO

```sh
pio run -t upload
```

See [platformio.ini](platformio.ini).

### Arduino IDE / MightyCore

This is plain C, not a sketch. Easiest path is `make`. If you must use the IDE,
create a sketch folder, copy `src/*.c` and `src/*.h` into it, rename `main.c`
to `<foldername>.ino`, and select *ATmega32 / External 16 MHz / Standard pinout*.
The `main()` here replaces Arduino's `setup()`/`loop()`, so also disable the
core's own main by selecting a bare-metal-friendly variant — `make` is simpler.

---

## 3. Controls — one button, three gestures

| Gesture | On a screen | In the menu | Editing a value |
|---|---|---|---|
| **Short press** | next screen | next item | next value |
| **Double press** | previous screen | leave menu | cancel the edit |
| **Hold** (0.6 s) | open menu | select / run | confirm + save |

The gesture map is also on the **Controls** screen (menu → Controls).

### Screens

1. **MONITOR** — large 7-segment BPM, SpO₂, perfusion index, live waveform strip,
   acquisition progress bar until the reading locks.
2. **WAVEFORM** — full-width PPG trace with HR/SpO₂ in the header.
3. **TRENDS** — 128 s scrolling history of HR and SpO₂, auto-scaled.
4. **ANALYSIS** — session time, beat count, HR min/avg/max, SpO₂ min, HRV
   (SDNN & RMSSD), respiration rate, sensor die temperature, quality index.
5. **SENSOR** — part/rev ID, measured sample rate, LED drive currents, DC and AC
   levels per channel, the ratio-of-ratios *R*, and the last inter-beat interval.

### Settings (persisted to EEPROM with a CRC)

LED drive (Auto/Low/Med/High) · sample averaging (1–32×) · SpO₂ trim (±5.0 %) ·
contrast · 180° flip · beat beep · auto-dim · **auto-sleep** · home screen ·
clear session · factory reset.

### Status LED (PB0)

PB0 has no timer output on the ATmega32, so brightness is a 32-level software
PWM in a short Timer2 overflow ISR — 7.8 kHz / 32 = 244 Hz, flicker-free, about
1.7 % CPU. The breathing curve is gamma-corrected (γ 2.2) so the fade looks
linear rather than snapping at the top.

| Pattern | Meaning |
|---|---|
| slow breathe | idle, waiting for a finger |
| double blip | finger present, still acquiring |
| dim + flash per beat | locked, flashing in time with your pulse |
| 4 Hz blink | sleep countdown running |
| two winks, then dark | going to sleep |
| one wink | woke up |

### Deep sleep

With **Auto Sleep** set (Off / 30 s / 1 min / 2 min / 5 min, default 2 min), the
idle timer runs whenever no finger is detected and no button is pressed. Nine
seconds before it expires a full-screen countdown appears — any press cancels it
and restarts the timer.

On expiry the firmware shuts things down in order: LED PWM, MAX30102 into
register-preserving shutdown (~0.7 µA), the OLED panel *and its charge pump*
(that DC-DC is what actually draws the milliamps), SPI, TWI, the 1 ms tick, and
finally the watchdog — which **must** go last and must go off, or it would reset
the board two seconds into the sleep. The MCU then enters power-down with a
low-level `INT0` armed on the button.

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
| `0x57 ID 0xNN need 15` | a chip answered at the MAX30102 address but reports a different part ID. **`0x11` means it is a MAX30100, not a MAX30102** — different register map, this firmware will not drive it. `0x15` covers MAX30102 and MAX30105. |
| `0x57 did not answer` | no ACK at the sensor's address. |

The header also shows the **bus speed actually in use**. `max30102_init()` now
tries 400 kHz, then bus recovery, then falls back to **100 kHz** and tries
again. Long dupont leads and weak pull-ups routinely fail at 400 kHz while
working perfectly at 100 kHz, so if the screen shows `I2C 100k` and the sensor
then works, that was your problem — shorten the wires or fit stronger pull-ups
(2.2 kΩ) if you want the faster bus back.

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

MAX30102 in SpO₂ mode: 411 µs pulse width (18-bit), 4096 nA ADC range, 400 Hz
with 4× on-chip averaging → **100 Hz effective**, both LEDs. The FIFO is drained
in one streaming I²C transaction that parses each 6-byte sample as it arrives,
so the driver needs a 6-byte buffer instead of a 192-byte one — which is what
makes this fit beside a 1 kB framebuffer on a 2 kB part.

### Signal chain (per sample, integer only)

```
raw ──▶ DC tracker (1-pole HP, 0.25 Hz) ──▶ HP2 (1-pole, 0.5 Hz)
    ──▶ LP × 2 (5 Hz) ──▶ band-limited PPG
```

Both channels get the **identical** filter, so the red/IR amplitude ratio that
SpO₂ depends on survives untouched while breathing and finger-movement wander is
removed.

### Beat detection

A **dual envelope** tracks the running peak *and* trough of the filtered IR
trace; the trigger sits at trough + ⅝ of the current pulse height and re-arms
below trough + ¼. Placing the threshold relative to the trough rather than to
zero is what stops residual baseline wander from masking every second beat —
without this the detector halves the reported rate under normal breathing.

The crossing instant is **linearly interpolated between samples**, so the
inter-beat interval has sub-sample resolution, which is what makes the HRV
figures meaningful. Intervals are gated to 300–2000 ms and rejected if they
deviate more than ⅓ from the running median; four consecutive rejections are
treated as a genuine rate change and reset the history.

### SpO₂

Per beat, peak-to-peak of the filtered red and IR traces gives the AC terms and
the DC trackers give the DC terms:

```
R = (AC_red / DC_red) / (AC_ir / DC_ir)
SpO2 = −45.06·R² + 30.354·R + 94.845      (evaluated in Q8 fixed point)
```

*R* is median-filtered over the last 8 beats before the curve is applied.

### Things that make it more accurate than the usual implementation

- **Sample-rate calibration.** The MAX30102's internal oscillator is only good
  to a few percent, and that error lands directly on BPM. The firmware counts
  actual samples against the 16 MHz crystal every 4 s and uses the *measured*
  rate to convert intervals to milliseconds.
- **Per-channel LED auto-gain.** Drive current is trimmed every 250 ms to hold
  each DC level in the ADC's linear region across skin tones and contact
  pressure. Samples during a gain step are discarded, and finger-detect
  thresholds scale with drive current so they stay valid at any gain.
- **PI from the raw signal, R from the filtered signal.** The low-pass
  attenuates the pulse by ~19 %, which would bias perfusion index low; it
  cancels exactly in *R* because both channels share the filter. So each metric
  is computed from whichever signal is correct for it.
- **Respiration from the baseline (RIIV), not from beat amplitudes.** Beat-
  amplitude methods sample once per beat — barely above Nyquist for breathing,
  and measured at ±3.5 breaths/min error. Band-passing the IR baseline instead
  samples at 100 Hz and costs three accumulators rather than a buffer.

### Validated accuracy

The fixed-point chain was ported to a reference model and run against synthetic
PPG with known ground truth (`ppg_shape` systolic peak + dicrotic notch, plus
respiratory modulation, baseline wander, Gaussian noise and HRV jitter):

| Test | Result |
|---|---|
| Heart rate, clean, 35–200 bpm | max error **0.4 bpm** |
| SpO₂ vs the Maxim curve, 74–100 % | max error **0.19 %** |
| HR under motion/breathing/noise (36 runs) | mean error **1.1 bpm**, worst 5.1 |
| SpO₂ under the same stress | mean error **0.60 %**, worst 2.11 |
| Beat rejection rate under stress | 4.7 % |
| Low-perfusion floor | works down to **PI 0.2 %** |
| Respiration, 8–30 breaths/min | **exact**; reports nothing when no respiratory signal is present |

These numbers characterise the *algorithm*, not the optics. Real-world accuracy
is dominated by sensor contact, ambient light and the fact that the SpO₂ curve
is uncalibrated for your specific module — see below.

### Calibrating SpO₂

The `−45.06R² + 30.354R + 94.845` curve is Maxim's generic reference. To trim
it for your hardware, compare against a reference oximeter on a healthy subject
at rest (where true SpO₂ is 96–99 %) and adjust **Settings → SpO₂ Trim**
(±5.0 % in 0.5 % steps, stored in EEPROM). This is a single-point offset — it
cannot correct the curve's shape at low saturation, and you cannot safely
generate low-saturation reference points at home.

---

## 6. Robustness

- 2 s watchdog, cleared once per main-loop pass; `MCUCSR` is captured and the
  WDT disabled in `.init3` so a watchdog reset cannot loop.
- Every TWI wait is bounded by a timeout — no infinite spin on a stuck bus.
- If the sensor stops responding, the UI shows a fault screen and the main loop
  runs bus recovery (9 clock pulses + STOP) and re-initialises once a second.
- Settings are validated by magic + version + CRC-8; a bad EEPROM falls back to
  defaults instead of loading garbage.
- The main loop never blocks: FIFO drain, DSP, housekeeping and a 20 fps redraw,
  with at most 8 samples consumed per pass so the button stays responsive.

---

## 7. Layout

| File | Purpose |
|---|---|
| [src/main.c](src/main.c) | startup, watchdog, main loop, sensor recovery |
| [src/config.h](src/config.h) | pin map and build-time options |
| [src/sys.c](src/sys.c) | 1 ms tick, `millis()`, button gesture recogniser |
| [src/i2c.c](src/i2c.c) | TWI master with timeouts + bus recovery |
| [src/max30102.c](src/max30102.c) | sensor driver, streaming FIFO reader, die temp |
| [src/ppg.c](src/ppg.c) | the signal chain — filters, beat detection, SpO₂, HRV, respiration, AGC |
| [src/ssd1306.c](src/ssd1306.c) | hardware SPI + panel driver |
| [src/gfx.c](src/gfx.c) | framebuffer primitives, 5×7 text, scalable 7-segment digits |
| [src/ui.c](src/ui.c) | screens, settings menu, single-button interaction |
| [src/settings.c](src/settings.c) | EEPROM persistence with CRC |
| [src/buzzer.c](src/buzzer.c) | optional beeper (compile-time gated) |
| [src/led.c](src/led.c) | status LED patterns, 32-level software PWM on Timer2 |
| [src/power.c](src/power.c) | deep sleep sequencing and wake-on-button |

---

## 8. Tuning

Everything worth touching is a `#define` at the top of
[src/ppg.c](src/ppg.c):

| Symbol | Effect |
|---|---|
| `HP2_SHIFT` | wander rejection vs. low-HR response (5 ≈ 0.5 Hz) |
| `ENV_DECAY` | how fast the trigger follows changing pulse height |
| `THR_HI_NUM` / `THR_LO_NUM` | trigger and re-arm points, in eighths of pulse height |
| `AMP_MIN_BEAT` | noise floor — raise it if you get false beats, lower it for weak pulses |
| `LOCK_BEATS` | beats required before a reading is published |
| `RESP_WIN` | respiration window (3000 samples = 30 s; first reading takes that long) |
| `SLEEP_COUNTDOWN_S` in [src/ui.c](src/ui.c) | how long the countdown runs before sleeping |
| `FINGER_ON_REF` / `FINGER_OFF_REF` | finger-detect thresholds at the reference LED current |
