#ifndef EMS_ENGINE_H
#define EMS_ENGINE_H

#include <stdint.h>
#include <stdbool.h>

/* ─── Mode Presets ─── */
typedef enum {
    EMS_MODE_SLEEP,
    EMS_MODE_RELAX,
    EMS_MODE_BREATH,
    EMS_MODE_CUSTOM,
    EMS_MODE_COUNT,
} ems_mode_t;

/* ─── FSM States ─── */
typedef enum {
    EMS_STATE_IDLE,
    EMS_STATE_BURST_ACTIVE,
    EMS_STATE_BURST_GAP,
    EMS_STATE_REST,
    EMS_STATE_PAUSED,
} ems_state_t;

/* ─── FSM Events ─── */
typedef enum {
    EMS_EVT_ENABLE,
    EMS_EVT_DISABLE,
    EMS_EVT_PAUSE,
    EMS_EVT_RESUME,
    EMS_EVT_TIMER_A,
    EMS_EVT_TIMER_B,
    EMS_EVT_BURST_STOPPED,
} ems_event_t;

/* ─── Stimulation Parameters ─── */
typedef struct {
    uint8_t  intensity;          /* 1-100 */
    uint8_t  pulse_width_pct;    /* 1-100 */
    uint32_t session_duration_s; /* 0 = infinite */
    ems_mode_t mode;

    uint16_t carrier_period_us;
    uint16_t carrier_on_us;      /* max at intensity=100 */
    uint16_t burst_duration_ms;
    uint16_t burst_interval_ms;
    uint16_t rest_duration_ms;
    uint16_t stim_duration_ms;
    uint8_t  inductor_offset_ms;
    uint8_t  stim_pulses;        /* auto-computed from inductor_offset */
    uint16_t stim_period_us;
} ems_params_t;

/* ─── Core API ─── */
int  ems_init(void);
/* Returns 0 on success, negative errno on failure. On failure, no PWM
 * peripherals are held and the engine remains in IDLE — caller must NOT
 * proceed with subsystem_activate / heartbeat / RR arming. */
int  ems_start(void);
void ems_stop(void);
/* Drive P0.0 HIGH to bleed residual inductor energy after the EMS rail
 * has been cut. Schedules P0.0 LOW after a short hold. Call ONLY after
 * ems_stop() AND after the EMS rail (LS1) has been disabled. */
void ems_discharge_after_stop(void);
void ems_pause(void);
void ems_resume(void);

/* ─── Parameter API ─── */
void ems_set_mode(ems_mode_t mode);
void ems_set_intensity(uint8_t intensity);
void ems_set_pulse_width(uint8_t pct);
void ems_set_session_duration(uint32_t secs);
void ems_apply_params_direct(const ems_params_t *params);
ems_params_t ems_get_params(void);
ems_state_t  ems_get_state(void);

/* ─── Callbacks (used by device FSM) ─── */
typedef void (*ems_cycle_cb_t)(void);
void ems_register_cycle_cb(ems_cycle_cb_t cb);
void ems_register_session_done_cb(ems_cycle_cb_t cb);
void ems_set_single_cycle(bool enable);

#endif
