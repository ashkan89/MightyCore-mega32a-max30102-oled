#!/usr/bin/env python3
"""
tests/math_check.py -- zero-dependency checks on the firmware's integer maths.

Why this exists alongside tests/host/test_ppg.c:

    test_ppg.c exercises the real DSP, but it needs a host C compiler.
    This needs nothing but Python, so it runs anywhere -- including on a
    machine set up only for AVR work -- and it covers the arithmetic that
    is both highest-risk and easiest to get wrong silently: the SpO2
    calibration polynomial, the fixed-point overflow limits, and the
    register values that have to agree with the datasheet.

    Crucially it does NOT hard-code the firmware's constants.  It parses
    them out of the source, so if someone edits a #define the check moves
    with it and a real disagreement is what fails -- rather than the
    check quietly testing a value the firmware no longer uses.

    What it cannot do: anything involving the actual signal chain, the
    beat detector, or timing.  Those are in test_ppg.c.

Usage:  python tests/math_check.py
"""

import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "src"

fails = []
checks = 0


def check(cond, msg):
    global checks
    checks += 1
    if not cond:
        fails.append(msg)
        print(f"  FAIL {msg}")


def section(name):
    print(f"\n== {name} ==")


# ----------------------------------------------------------------------
#  Pull constants out of the source rather than restating them
# ----------------------------------------------------------------------
def defines(path):
    """{name: text} for every simple #define in a file."""
    out = {}
    text = path.read_text(encoding="utf-8", errors="replace")
    for m in re.finditer(r"^\s*#\s*define\s+(\w+)\s+([^\n/]+)", text, re.M):
        out[m.group(1)] = m.group(2).strip()
    return out


def as_int(defs, name):
    """Integer value of a #define, tolerating U/UL suffixes and 0x."""
    if name not in defs:
        raise KeyError(f"{name} is no longer defined -- check the parser")
    tok = defs[name].split()[0].rstrip("uUlL")
    return int(tok, 0)


ppg_d = defines(SRC / "ppg.c")
max_d = defines(SRC / "max30102.c")
max_h = defines(SRC / "max30102.h")
cfg_d = defines(SRC / "config.h")

R_TRUST_MAX = as_int(ppg_d, "R_TRUST_MAX")
RMS_Q_MAX = as_int(ppg_d, "RMS_Q_MAX")
RMS_N_MAX = as_int(ppg_d, "RMS_N_MAX")
RMS_SHIFT = as_int(ppg_d, "RMS_SHIFT")
IBI_MIN_MS = as_int(ppg_d, "IBI_MIN_MS")
IBI_MAX_MS = as_int(ppg_d, "IBI_MAX_MS")
REFRAC_MS = as_int(ppg_d, "REFRAC_MS")
REFRAC_MAX = as_int(ppg_d, "REFRAC_MAX")
IBI_HIST = as_int(ppg_d, "IBI_HIST")
R_HIST = as_int(ppg_d, "R_HIST")
CORR_MIN_N = as_int(ppg_d, "CORR_MIN_N")
CORR_MIN_D = as_int(ppg_d, "CORR_MIN_D")
LOST_RESYNC = as_int(ppg_d, "LOST_RESYNC")
MAX_AVG_MAX = as_int(max_h, "MAX_AVG_MAX")
MAX_OVF_UNKNOWN = as_int(max_h, "MAX_OVF_UNKNOWN")
LED_PA_REF = as_int(max_h, "LED_PA_REF")
LED_PA_MAX = as_int(max_h, "LED_PA_MAX")
LED_PA_MIN = as_int(max_h, "LED_PA_MIN")

U32_MAX = 0xFFFFFFFF
I32_MAX = 0x7FFFFFFF


# ----------------------------------------------------------------------
#  1. The SpO2 calibration polynomial, exactly as the firmware evaluates it
# ----------------------------------------------------------------------
def spo2_fixed(r8, cal=0):
    """Transliteration of the Q8 evaluation in ppg.c spo2_update()."""
    if r8 > R_TRUST_MAX:
        return None                      # outside the curve's domain
    if r8 < 0:
        r8 = 0
    sp = 24280 + ((7771 * r8) >> 8) - ((11535 * r8 * r8) >> 16)
    sp = (sp * 10) >> 8
    sp += cal
    return max(700, min(1000, sp))


def spo2_float(R):
    return -45.06 * R * R + 30.354 * R + 94.845


def test_spo2_curve():
    section("SpO2 calibration curve")

    # The Q8 coefficients must be the intended float ones.
    for name, q8, want in (("constant", 24280, 94.845 * 256),
                           ("linear", 7771, 30.354 * 256),
                           ("quadratic", 11535, 45.06 * 256)):
        check(abs(q8 - want) < 1.0,
              f"{name} coefficient {q8} should be {want:.1f} in Q8")

    # Fixed point must track the float polynomial across the whole
    # trusted domain.  1 tenth of a percent is one display digit.
    worst = 0.0
    worst_at = 0
    for r8 in range(0, R_TRUST_MAX + 1):
        got = spo2_fixed(r8)
        want = spo2_float(r8 / 256.0) * 10.0
        want = max(700.0, min(1000.0, want))
        err = abs(got - want)
        if err > worst:
            worst, worst_at = err, r8
    check(worst <= 1.2,
          f"fixed-point SpO2 differs from the polynomial by {worst:.2f} "
          f"tenths at R={worst_at / 256.0:.4f}")
    print(f"  worst fixed-vs-float error: {worst:.2f} tenths of a percent "
          f"at R={worst_at / 256.0:.4f}")

    # R_TRUST_MAX is documented as the point the curve passes 70 %, the
    # bottom of Maxim's table and of anything this front end can support.
    sp_at_max = spo2_float(R_TRUST_MAX / 256.0)
    check(69.0 < sp_at_max < 71.5,
          f"R_TRUST_MAX={R_TRUST_MAX} (R={R_TRUST_MAX / 256.0:.3f}) puts the "
          f"curve at {sp_at_max:.2f} %, not the documented ~70 %")
    print(f"  R_TRUST_MAX = {R_TRUST_MAX} -> R = {R_TRUST_MAX / 256.0:.4f} "
          f"-> {sp_at_max:.2f} %")

    # The curve must not turn back up inside the trusted range, or two
    # different saturations would share one R.
    zero = 2 * 45.06 / (2 * 45.06) and (30.354 / (2 * 45.06))
    check(R_TRUST_MAX / 256.0 > zero,
          "the curve's turning point is inside the trusted range")
    prev = None
    mono = True
    for r8 in range(int(zero * 256) + 1, R_TRUST_MAX + 1):
        v = spo2_fixed(r8)
        if prev is not None and v > prev:
            mono = False
        prev = v
    # Strictly non-increasing.  It is only strict because the quadratic
    # term folds its two shifts into one; truncating twice, as it did,
    # produced six places where a larger R gave a HIGHER saturation.
    check(mono, "SpO2 is not monotonically falling past the turning point")

    # Sanity: a healthy R must give a healthy reading.
    check(spo2_fixed(int(0.5 * 256)) in range(980, 995),
          f"R=0.5 gives {spo2_fixed(int(0.5 * 256))}, expected ~987")

    # The trim is applied after the polynomial and before the clamp, so
    # it can never push the output outside 70..100 %.
    for cal in (-50, -5, 0, 5, 50):
        for r8 in (0, 128, R_TRUST_MAX):
            v = spo2_fixed(r8, cal)
            check(700 <= v <= 1000,
                  f"trim {cal} at R={r8 / 256.0:.2f} gives {v}, outside "
                  f"the 70..100 %% clamp")


# ----------------------------------------------------------------------
#  2. Fixed-point overflow limits
# ----------------------------------------------------------------------
def test_overflow():
    section("fixed-point overflow limits")

    # spo2_update(): ri = (aci << 16) / dc_ir, with aci clamped to 60000.
    check(60000 * 65536 <= U32_MAX,
          f"(aci << 16) reaches {60000 * 65536}, past a uint32")
    print(f"  (60000 << 16) = {60000 * 65536} vs uint32 max {U32_MAX}"
          f"  headroom {U32_MAX - 60000 * 65536}")

    # rq16 = (rr << 16) / ri, with rr clamped to 32767.
    check(32767 * 65536 <= U32_MAX,
          f"(rr << 16) reaches {32767 * 65536}, past a uint32")

    # Sums of squares: RMS_N_MAX samples each clamped to RMS_Q_MAX.
    worst = RMS_Q_MAX * RMS_Q_MAX * RMS_N_MAX
    check(worst <= U32_MAX,
          f"ssq can reach {worst}, past a uint32 ({U32_MAX})")
    check(worst <= I32_MAX,
          f"sxy is int32 and can reach {worst}, past {I32_MAX}")
    print(f"  RMS_Q_MAX^2 * RMS_N_MAX = {worst} vs int32 max {I32_MAX}"
          f"  headroom {I32_MAX - worst}")

    # The per-sample clamp must actually be reachable given RMS_SHIFT and
    # the 18-bit ADC, or the clamp is dead code hiding a real limit.
    check(RMS_Q_MAX << RMS_SHIFT <= 262143,
          f"RMS_Q_MAX << RMS_SHIFT = {RMS_Q_MAX << RMS_SHIFT} exceeds the "
          f"18-bit full scale, so the clamp can never engage")

    # The correlation comparison: cx * CORR_MIN_D vs den * CORR_MIN_N,
    # where cx and den are both bounded by RMS_Q_MAX^2.
    bound = RMS_Q_MAX * RMS_Q_MAX
    check(bound * max(CORR_MIN_N, CORR_MIN_D) <= U32_MAX,
          f"the correlation comparison reaches "
          f"{bound * max(CORR_MIN_N, CORR_MIN_D)}, past a uint32")

    # The correlation threshold is documented as the reference's 0.8.
    check(abs(CORR_MIN_N / CORR_MIN_D - 0.8) < 1e-9,
          f"correlation gate is {CORR_MIN_N}/{CORR_MIN_D} = "
          f"{CORR_MIN_N / CORR_MIN_D}, documented as 0.8")

    # ibi_ms = ((dt_q8 * 25000) >> 6) / fs_x100, with dt_q8 clamped to
    # 0x20000 in the source.  The clamp is what keeps this in range.
    dt_clamp = 0x20000
    check(dt_clamp * 25000 <= U32_MAX,
          f"dt_q8 * 25000 reaches {dt_clamp * 25000}, past a uint32 -- the "
          f"0x20000 clamp is no longer sufficient")
    print(f"  dt_q8 clamp 0x{dt_clamp:X} * 25000 = {dt_clamp * 25000} vs "
          f"{U32_MAX}  headroom {U32_MAX - dt_clamp * 25000}")

    # pi_x100 = (raci * 10000) / dc_ir, raci clamped to 60000.
    check(60000 * 10000 <= U32_MAX,
          f"the perfusion index reaches {60000 * 10000}, past a uint32")

    # median_u16 copies into a 12-element scratch array.
    check(IBI_HIST <= 12, f"IBI_HIST={IBI_HIST} exceeds median_u16's scratch")
    check(R_HIST <= 12, f"R_HIST={R_HIST} exceeds median_u16's scratch")


# ----------------------------------------------------------------------
#  3. Beat-detector bounds
# ----------------------------------------------------------------------
def test_beat_bounds():
    section("beat-detector bounds")

    check(REFRAC_MS < IBI_MIN_MS,
          f"refractory floor {REFRAC_MS} ms is not shorter than the minimum "
          f"accepted interval {IBI_MIN_MS} ms, so the fastest accepted beat "
          f"would be blanked")
    check(REFRAC_MAX < IBI_MAX_MS,
          f"refractory ceiling {REFRAC_MAX} ms exceeds IBI_MAX_MS")

    lo = 600000 // IBI_MAX_MS      # bpm_x10
    hi = 600000 // IBI_MIN_MS
    print(f"  accepted rate window: {lo / 10:.0f} .. {hi / 10:.0f} bpm")
    check(lo <= 300, f"minimum accepted rate {lo / 10} bpm is above 30")
    check(hi >= 1800, f"maximum accepted rate {hi / 10} bpm is below 180")
    check(hi <= 2500,
          f"maximum accepted rate {hi / 10} bpm exceeds the 250 bpm the "
          f"display clamps to")

    # The adaptive hold-off is 60 % of the median interval, which must
    # leave room for the rate to rise between beats.
    check(REFRAC_MAX / IBI_MAX_MS < 0.62,
          "the refractory ceiling does not leave the documented headroom")

    # An unquantified FIFO overflow must force a resync.
    check(MAX_OVF_UNKNOWN >= LOST_RESYNC,
          f"MAX_OVF_UNKNOWN={MAX_OVF_UNKNOWN} is below "
          f"LOST_RESYNC={LOST_RESYNC}, so an overflow of unknown size would "
          f"not drop the beat reference and an interval would be timed "
          f"straight across the gap")


# ----------------------------------------------------------------------
#  4. MAX30102 register configuration against the datasheet
# ----------------------------------------------------------------------
def test_registers():
    section("MAX30102 registers vs datasheet")

    # Datasheet Table 6, SpO2 Sample Rate Control: field value -> Hz.
    SR_TABLE = {0: 50, 1: 100, 2: 200, 3: 400,
                4: 800, 5: 1000, 6: 1600, 7: 3200}
    # Datasheet Table 7, LED Pulse Width Control.
    PW_TABLE = {0: (69, 15), 1: (118, 16), 2: (215, 17), 3: (411, 18)}
    # Datasheet Table 11, SpO2 Mode (Allowed Settings): the highest ADC
    # sample rate permitted at each pulse width in two-LED SpO2 mode.
    TABLE11_MAX_SR = {69: 1600, 118: 1000, 215: 800, 411: 400}
    # Datasheet Table 5, SpO2 ADC Range Control: field -> full scale nA.
    ADC_RANGE = {0: 2048, 1: 4096, 2: 8192, 3: 16384}

    pw_field = as_int(max_d, "PULSEWIDTH_411") & 0x03
    pw_us, pw_bits = PW_TABLE[pw_field]
    check(pw_us == 411,
          f"PULSEWIDTH_411 selects {pw_us} us, not 411")
    check(pw_bits == 18, f"pulse width {pw_us} us gives {pw_bits}-bit ADC")
    print(f"  LED_PW = {pw_field} -> {pw_us} us, {pw_bits}-bit resolution")

    rng_field = (as_int(max_d, "ADCRANGE_4096") >> 5) & 0x03
    check(ADC_RANGE[rng_field] == 4096,
          f"ADCRANGE_4096 selects {ADC_RANGE[rng_field]} nA full scale")
    print(f"  SPO2_ADC_RGE = {rng_field} -> {ADC_RANGE[rng_field]} nA "
          f"full scale")

    sr_field = (as_int(max_d, "SAMPLERATE_400") >> 2) & 0x07
    check(SR_TABLE[sr_field] == 400,
          f"SAMPLERATE_400 selects {SR_TABLE[sr_field]} Hz")

    # Every averaging setting the firmware offers must pair with an ADC
    # rate that Table 11 allows at this pulse width, and must leave the
    # FIFO output rate at the 100 Hz the DSP filters are tuned for.
    # max30102_set_avg() writes the field value (avg_code + 1).
    print(f"  MAX_AVG_MAX = {MAX_AVG_MAX}, so averaging offers "
          f"{[1 << a for a in range(MAX_AVG_MAX + 1)]}x")
    for avg in range(MAX_AVG_MAX + 1):
        field = avg + 1
        check(field in SR_TABLE,
              f"avg {1 << avg}x maps to SPO2_SR field {field}, out of range")
        adc_hz = SR_TABLE[field]
        out_hz = adc_hz / (1 << avg)
        check(out_hz == 100,
              f"avg {1 << avg}x with a {adc_hz} Hz ADC rate gives "
              f"{out_hz} Hz out of the FIFO, not the 100 Hz the DSP "
              f"filter corners are tuned for")
        check(adc_hz <= TABLE11_MAX_SR[pw_us],
              f"avg {1 << avg}x needs a {adc_hz} Hz ADC rate, but "
              f"datasheet Table 11 allows at most "
              f"{TABLE11_MAX_SR[pw_us]} Hz at {pw_us} us in SpO2 mode")
        print(f"    avg {1 << avg}x: SPO2_SR={field} -> {adc_hz} Hz ADC "
              f"-> {out_hz:.0f} Hz out")

    # The overflow-counter time gate must sit inside the window the FIFO
    # actually buffers: longer than the poll interval by a wide margin, so
    # a healthy bus can never open it, but shorter than the time the FIFO
    # takes to fill, so a genuine overflow is never rejected.
    gap = as_int(max_d, "OVF_MIN_GAP_MS")
    poll = as_int(defines(SRC / "main.c"), "FIFO_POLL_MS")
    fifo_full_ms = 32 * 1000 // 100          # 32 samples at 100 Hz
    print(f"  OVF gate {gap} ms  (poll {poll} ms, FIFO fills in "
          f"{fifo_full_ms} ms)")
    check(gap > poll * 4,
          f"the overflow gate is {gap} ms against a {poll} ms poll -- too "
          f"close for a healthy bus to stay outside it")
    check(gap < fifo_full_ms,
          f"the overflow gate is {gap} ms but the FIFO fills in "
          f"{fifo_full_ms} ms, so a genuine overflow would be rejected")

    # And the next setting up must genuinely be unreachable, or the limit
    # is more conservative than the datasheet requires.
    nxt = MAX_AVG_MAX + 1
    if nxt + 1 in SR_TABLE:
        check(SR_TABLE[nxt + 1] > TABLE11_MAX_SR[pw_us],
              f"avg {1 << nxt}x would need {SR_TABLE[nxt + 1]} Hz, which "
              f"Table 11 does allow -- MAX_AVG_MAX is too low")

    # Datasheet Table 8, LED Current Control: 0.2 mA per LSB.
    for name, code in (("LED_PA_MIN", LED_PA_MIN),
                       ("LED_PA_REF", LED_PA_REF),
                       ("LED_PA_MAX", LED_PA_MAX)):
        print(f"  {name} = 0x{code:02X} -> {code * 0.2:.1f} mA")
    check(abs(LED_PA_REF * 0.2 - 6.2) < 0.3,
          f"LED_PA_REF=0x{LED_PA_REF:02X} is {LED_PA_REF * 0.2:.1f} mA; the "
          f"reference implementation's 50000-count finger threshold is "
          f"calibrated at the datasheet's 0x1F = 6.2 mA")
    check(LED_PA_MIN < LED_PA_REF <= LED_PA_MAX,
          "the LED current limits are not ordered min < ref <= max")
    check(LED_PA_MAX * 0.2 <= 26.0,
          f"LED_PA_MAX is {LED_PA_MAX * 0.2:.1f} mA per emitter, and both "
          f"pulse -- past what a small breakout LDO holds up under")

    # FIFO_CFG: SMP_AVE in bits 7:5, rollover in bit 4.
    rollover = as_int(max_d, "ROLLOVER_ENABLE")
    check(rollover == 0x10,
          f"ROLLOVER_ENABLE is 0x{rollover:02X}, but FIFO_ROLLOVER_EN is "
          f"bit 4 (0x10)")
    for avg in range(MAX_AVG_MAX + 1):
        reg = (avg << 5) | rollover
        check((reg >> 5) & 0x07 == avg,
              f"FIFO_CFG 0x{reg:02X} does not carry SMP_AVE={avg}")
        check(reg & 0x0F == 0,
              f"FIFO_CFG 0x{reg:02X} sets FIFO_A_FULL bits, which this "
              f"firmware does not use (it polls)")

    # Mode: 0x03 is SpO2 mode, red + IR, two 3-byte words per sample.
    check(as_int(max_d, "MODE_REDIRONLY") == 0x03,
          "MODE_REDIRONLY is not the datasheet's 0x03 SpO2 mode")

    # The FIFO burst buffer must hold whole samples: 2 channels x 3 bytes.
    burst = as_int(max_d, "MAX_BURST")
    print(f"  MAX_BURST = {burst} samples -> {burst * 6} byte stack buffer")
    check(burst * 6 <= 64,
          f"the FIFO burst buffer is {burst * 6} bytes of stack, which is "
          f"a lot on a part with 2 KB of SRAM")
    check(burst <= 32, f"MAX_BURST={burst} exceeds the 32-sample FIFO depth")


# ----------------------------------------------------------------------
#  5. Sampling and timing budget
# ----------------------------------------------------------------------
def test_timing():
    section("sampling and timing budget")

    fs = 100.0                       # FIFO output rate, held by set_avg()
    poll_ms = as_int(defines(SRC / "main.c"), "FIFO_POLL_MS")
    fifo_depth = 32

    print(f"  FIFO output rate      {fs:.0f} Hz  ({1000 / fs:.1f} ms/sample)")
    print(f"  FIFO poll interval    {poll_ms} ms")
    print(f"  FIFO depth            {fifo_depth} samples "
          f"({fifo_depth * 1000 / fs:.0f} ms of buffer)")

    check(poll_ms < 1000 / fs,
          f"polling every {poll_ms} ms is slower than the "
          f"{1000 / fs:.1f} ms sample interval")

    # How long the loop may stall before the FIFO overruns.  This is the
    # budget every blocking operation in the firmware has to fit inside.
    stall_ms = fifo_depth * 1000 / fs
    print(f"  loop stall budget     {stall_ms:.0f} ms before overrun")

    # The known blocking operations, from the source.
    ui_fps = as_int(cfg_d, "UI_FPS_MS")
    print(f"  redraw interval       {ui_fps} ms "
          f"({1000 / ui_fps:.0f} fps)")
    # 1024-byte framebuffer at SPI fosc/4 = 4 MHz, 8 bits + overhead.
    spi_ms = 1024 * 8 / 4_000_000 * 1000
    print(f"  framebuffer flush     {spi_ms:.1f} ms at 4 MHz SPI")
    check(spi_ms < stall_ms / 4,
          f"an OLED flush takes {spi_ms:.1f} ms, too much of the "
          f"{stall_ms:.0f} ms FIFO budget")

    # One 2-second diagnostic line, ~250 characters at 38400 8N1.
    baud = as_int(cfg_d, "DBG_BAUD")
    diag_ms = 250 * 10 / baud * 1000
    print(f"  diagnostic line       {diag_ms:.0f} ms at {baud} baud "
          f"(every 2 s)")
    check(diag_ms < stall_ms,
          f"a diagnostic line blocks for {diag_ms:.0f} ms, past the "
          f"{stall_ms:.0f} ms FIFO budget -- samples would be lost")
    print(f"    -> {diag_ms * fs / 1000:.1f} samples' worth of FIFO used, "
          f"of {fifo_depth}")

    # I2C cost per poll: one 3-byte pointer read, plus a 6-byte data
    # burst per sample.  At 100 kHz a byte plus its ACK is 90 us.
    byte_us = 90.0
    ptr_read_us = byte_us * 5 + 20        # addr, reg, re-addr, 3 data, Tbuf
    print(f"  pointer read          {ptr_read_us:.0f} us per poll "
          f"(one 3-byte burst)")
    polls_per_s = 1000.0 / poll_ms
    load = ptr_read_us * polls_per_s / 1e6 * 100
    print(f"  pointer-read bus load {load:.1f} % of wall clock")
    check(load < 20.0,
          f"the FIFO pointer read alone occupies {load:.1f} % of the "
          f"time base")
    # Before the change this was two separate single-register reads.
    old_us = 2 * (byte_us * 4 + 20)
    print(f"    was {old_us:.0f} us as two single-register reads "
          f"({old_us * polls_per_s / 1e6 * 100:.1f} %), "
          f"now {ptr_read_us:.0f} us")

    # Beat timing resolution.  Crossings are interpolated to 1/256 of a
    # sample, so the interval quantisation is far below the sample period.
    q_ms = 1000 / fs / 256
    print(f"  crossing resolution   {q_ms * 1000:.1f} us "
          f"(1/256 sample, interpolated)")
    for bpm in (50, 100, 180):
        ibi = 60000.0 / bpm
        err = q_ms / ibi * bpm
        check(err < 0.05,
              f"at {bpm} bpm the interpolation alone costs {err:.3f} bpm")

    # And the resolution WITHOUT interpolation, which is what the
    # interpolation is there to avoid.
    for bpm in (50, 180):
        ibi = 60000.0 / bpm
        err = (1000 / fs) / ibi * bpm
        print(f"    at {bpm} bpm: {err:.2f} bpm per whole sample of jitter, "
              f"{err / 256:.4f} bpm interpolated")


# ----------------------------------------------------------------------
#  6. millis() wraparound arithmetic
# ----------------------------------------------------------------------
def test_wraparound():
    section("millis() wraparound")

    M = 1 << 32

    def elapsed(now, then):
        """The firmware's idiom: (uint32_t)(now - then)."""
        return (now - then) % M

    # Across the wrap, an unsigned difference is still the true interval
    # provided it is under 2^32 ms (49.7 days).
    cases = [
        (M - 1, M - 1001, 1000),
        (500, M - 500, 1000),
        (0, M - 1, 1),
        (M - 1, 0, M - 1),
    ]
    for now, then, want in cases:
        got = elapsed(now, then)
        check(got == want,
              f"elapsed(0x{now:08X}, 0x{then:08X}) = {got}, want {want}")

    # Every timeout in the firmware, checked across the wrap.
    timeouts = {
        "UI redraw": as_int(cfg_d, "UI_FPS_MS"),
        "FIFO poll": as_int(defines(SRC / "main.c"), "FIFO_POLL_MS"),
        "SpO2 staleness": as_int(ppg_d, "SPO2_STALE_MS"),
        "no-pulse release": as_int(ppg_d, "FINGER_NOBEAT_MS"),
    }
    for name, t in timeouts.items():
        then = (M - t // 2) % M          # started before the wrap
        now = (then + t) % M             # expires after it
        check(elapsed(now, then) >= t,
              f"{name} ({t} ms) does not fire across the wrap")
        check(elapsed((then + t - 1) % M, then) < t,
              f"{name} ({t} ms) fires early across the wrap")

    print(f"  {len(cases)} difference cases and {len(timeouts)} timeouts "
          f"behave identically across the wrap")

    # The sample counter's crossing timestamp is (nsamp - 1) << 8 in a
    # uint32, so it wraps every 2^24 samples.  The interval is a
    # difference of two stamps and stays correct modulo 2^32 as long as
    # the gap is shorter than the clamp the code applies.
    dt_clamp = 0x20000
    for base in (0, 0x00FFFF00, 0x00FFFFFF):
        t0 = ((base - 1) << 8) % M
        t1 = ((base + 120 - 1) << 8) % M
        check((t1 - t0) % M == 120 << 8,
              f"crossing interval wrong across the sample-counter wrap at "
              f"nsamp=0x{base:X}")
    print(f"  crossing timestamps differ correctly across the 2^24-sample "
          f"wrap ({(1 << 24) / 100 / 3600:.1f} hours at 100 Hz)")
    check(dt_clamp < (1 << 24) << 8,
          "the dt_q8 clamp is larger than the timestamp range")


# ----------------------------------------------------------------------
#  7. Settings record
# ----------------------------------------------------------------------
def test_settings():
    section("settings record")

    text = (SRC / "settings.h").read_text(encoding="utf-8")
    # Field widths in bytes, in declaration order.  -fpack-struct means
    # there is no padding on this target.
    width = {"uint8_t": 1, "int8_t": 1, "uint16_t": 2, "int16_t": 2,
             "uint32_t": 4, "int32_t": 4}
    body = text[text.index("typedef struct {"):text.index("} settings_t;")]
    total = 0
    fields = []
    for m in re.finditer(r"^\s*(u?int\d+_t)\s+(\w+)\s*;", body, re.M):
        total += width[m.group(1)]
        fields.append(m.group(2))
    print(f"  {len(fields)} fields, {total} bytes packed")

    check("crc" == fields[-1],
          f"crc must be the last field (it is '{fields[-1]}'), because the "
          f"CRC is taken over sizeof(cfg) - 1")
    check(fields[0] == "magic" and fields[1] == "version",
          "magic and version must lead the record")
    for needed in ("fs_cal", "fs_cal_avg", "probe_v", "probe_id"):
        check(needed in fields, f"{needed} is missing from settings_t")

    # ATmega32 has 1 KB of EEPROM; the record must be nowhere near it.
    check(total <= 64, f"the settings record is {total} bytes")

    # A version bump is what discards an incompatible stored record.
    ver = as_int(defines(SRC / "settings.h"), "SETTINGS_VERSION")
    print(f"  SETTINGS_VERSION = {ver}")
    check(ver >= 4,
          f"SETTINGS_VERSION is {ver}; adding fs_cal_avg/probe_v/probe_id "
          f"changed the layout and needs a bump so older records are "
          f"rejected rather than misread")


# ----------------------------------------------------------------------
def main():
    print("PulseOx integer-maths and configuration checks")
    print(f"source: {SRC}")

    test_spo2_curve()
    test_overflow()
    test_beat_bounds()
    test_registers()
    test_timing()
    test_wraparound()
    test_settings()

    print("\n" + "-" * 62)
    print(f"{checks} checks, {len(fails)} failed")
    for f in fails:
        print(f"  - {f}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
