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
    uint16_t r_q12;         /* ratio-of-ratios R in Q12                   */
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

void ppg_init(void);
void ppg_reset_measure(void);       /* clears the running measurement      */
void ppg_reset_session(void);       /* clears session min/max/duration     */
void ppg_process(uint32_t red, uint32_t ir);   /* one FIFO sample          */
void ppg_lost_samples(uint8_t n);   /* keeps the sample time base honest   */
void ppg_service(void);             /* AGC, temperature, fs calibration    */

/* live waveform ring, filled at fs/2 so 128 px spans about 2.5 s */
extern int8_t  ppg_wave[WAVE_LEN];
extern uint8_t ppg_wave_head;

#endif
