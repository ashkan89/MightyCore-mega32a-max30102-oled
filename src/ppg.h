/* ppg.h -- photoplethysmography signal chain: DC tracking, band-limiting,
 *          adaptive beat detection, ratio-of-ratios SpO2, HRV, respiration,
 *          LED auto-gain and true sample-rate calibration.
 *
 * All fixed point, O(1) memory per metric: nothing here allocates a sample
 * window, which is what makes it fit next to a 1 KB framebuffer on a 2 KB part.
 */
#ifndef PPG_H
#define PPG_H
#include "config.h"

typedef struct {
    /* --- presence / quality --- */
    uint8_t  finger;        /* 1 = finger detected                        */
    uint8_t  valid;         /* 1 = BPM/SpO2 have converged                */
    uint8_t  progress;      /* 0..100 acquisition progress                */
    uint8_t  sqi;           /* 0..100 signal quality index                */

    /* --- primary vitals --- */
    uint16_t bpm_x10;       /* heart rate, tenths of a bpm                */
    uint16_t spo2_x10;      /* SpO2, tenths of a percent                  */
    uint16_t pi_x100;       /* perfusion index, hundredths of a percent   */
    uint16_t ibi_ms;        /* last accepted inter-beat interval          */

    /* --- derived --- */
    uint16_t sdnn_ms;       /* HRV: SD of NN intervals                    */
    uint16_t rmssd_ms;      /* HRV: RMS of successive differences         */
    uint8_t  resp_bpm;      /* respiration estimate, breaths/min          */
    int16_t  temp_x10;      /* MAX30102 die temperature                   */

    /* --- raw / diagnostics --- */
    uint32_t dc_ir, dc_red;
    uint32_t finger_th;     /* live finger-detect threshold, for display   */
    uint32_t base_ir;       /* learned no-finger IR level, for display     */
    uint32_t refl_ir;       /* IR DC normalised to the reference LED drive */
    uint16_t ac_ir, ac_red;
    /* The band-passed AC spans that actually form R, as opposed to ac_ir /
     * ac_red above, which are the RAW spans and only feed the perfusion
     * index.  Reported separately because a wrong R is nearly always one
     * channel's AC being wrong, and the raw figures cannot show which. */
    uint16_t fac_ir, fac_red;
    uint16_t r_q12;         /* ratio-of-ratios R in Q12                   */
    /* Why the last completed beat window produced no saturation -- one of
     * the SPO2_* codes below.  Every path out of spo2_update() sets it, so
     * "no SpO2" is always an answerable question rather than a blank field:
     * three of these used to return silently, and a device that will not
     * measure looked identical whether the red return was dead, the pulse
     * was too weak or the ratio had simply landed off the curve.
     *
     * It describes the LAST WINDOW, not the displayed value: a published
     * reading survives the odd bad window and is retired by the staleness
     * timer in ppg_service(), so a non-zero code with spo2_x10 still set
     * means "this beat gave nothing", not "there is no reading". */
    uint8_t  spo2_rail;
    uint8_t  corr_x100;     /* red/IR Pearson correlation, 0..100          */
    uint8_t  led_ir, led_red;
    uint16_t fs_x100;       /* calibrated sample rate x100, used for timing */
    uint16_t sps;           /* raw samples actually received per second     */
    uint32_t beats;         /* accepted beats this session                */
    uint32_t rejects;
    int8_t   wave;          /* normalised waveform sample, -110..110      */
    uint8_t  beat;          /* pulses to 1 on each accepted beat          */
    uint32_t beat_ms;       /* millis() of last beat                      */

    /* --- session statistics --- */
    uint16_t bpm_min_x10, bpm_max_x10, bpm_avg_x10;
    uint16_t spo2_min_x10;
    uint32_t sess_start_ms;
} ppg_state_t;

extern ppg_state_t ppg;

/* ppg_state_t.spo2_rail -- why the last beat window published nothing.
 *
 *   SPO2_OK        a reading was published from that window
 *   SPO2_R_RANGE   R past the curve's calibrated domain.  Not a low
 *                  saturation: the channels are reversed, or something
 *                  non-pulsatile is riding on one of them.  r_q12 is the
 *                  diagnosis
 *   SPO2_CORR      RED and IR did not move together, so the pair does not
 *                  describe a pulse whatever R comes out of it.  Dead or
 *                  unseen red emitter, movement, ambient light.
 *                  corr_x100 is the diagnosis
 *   SPO2_WEAK      one channel's band-passed AC span is too small to form
 *                  a ratio from.  fac_ir / fac_red say which
 *   SPO2_DC        DC too low to divide by -- the finger is barely on the
 *                  sensor, or an emitter is not driven at all
 *   SPO2_WARMUP    not enough correlation history yet.  Transient, and
 *                  only ever seen in the first seconds of a measurement
 */
#define SPO2_OK        0
#define SPO2_R_RANGE   1
#define SPO2_CORR      2
#define SPO2_WEAK      3
#define SPO2_DC        4
#define SPO2_WARMUP    5

void ppg_init(void);
void ppg_reset_measure(void);       /* clears the running measurement      */
void ppg_reset_session(void);       /* clears session min/max/duration     */
void ppg_process(uint32_t red, uint32_t ir);   /* one FIFO sample          */
void ppg_lost_samples(uint8_t n);   /* keeps the sample time base honest   */
void ppg_service(void);             /* AGC, temperature, fs calibration    */
/* Call after cfg.avg_code changes.  Re-derives every rate-dependent filter
 * shift and window length, and discards the sample-rate calibration that
 * was measured at the old setting. */
void ppg_rate_changed(void);

/* live waveform ring, filled at fs/2 so 128 px spans about 2.5 s */
extern int8_t  ppg_wave[WAVE_LEN];
extern uint8_t ppg_wave_head;

#endif
