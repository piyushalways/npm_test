#include "ems_engine.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <nrfx_pwm.h>
#include <nrfx_grtc.h>
#include <hal/nrf_gpio.h>
#include <string.h>

/*
 * PWM timing macros — normally from <zephyr/drivers/pwm.h> but only available
 * when CONFIG_PWM=y.  We use nrfx directly (CONFIG_PWM=n), so define them here.
 */
#ifndef PWM_MSEC
#define PWM_MSEC(ms)   ((uint64_t)(ms) * 1000000ULL)
#endif
#ifndef PWM_USEC
#define PWM_USEC(us)   ((uint64_t)(us) * 1000ULL)
#endif
#ifndef NSEC_PER_USEC
#define NSEC_PER_USEC  1000UL
#endif

/* ═══════════════════════════════════════════════════════════════════
 *  Pin Definitions
 * ═══════════════════════════════════════════════════════════════════ */

#define CHAN0_GPIO_PIN       NRF_GPIO_PIN_MAP(0, 0)   /* P0.0  — safety GPIO (DPPI kill) */
#define CHAN1_PWM_GPIO_PIN   NRF_GPIO_PIN_MAP(1, 13)  /* P1.13 — PWM20 carrier */
#define CHAN2_PWM_GPIO_PIN   NRF_GPIO_PIN_MAP(1, 11)  /* P1.11 — PWM21 ch0 stim (unchanged) */
#define CHAN3_PWM_GPIO_PIN   NRF_GPIO_PIN_MAP(1, 7)   /* P1.07 — PWM22 ch1 stim */

#define CHAN1_PWM_INST       NRFX_PWM_INSTANCE(20)
#define CHAN2_PWM_INST       NRFX_PWM_INSTANCE(21)   /* P1.11 stim ch2 */
#define CHAN3_PWM_INST       NRFX_PWM_INSTANCE(22)   /* P1.07 stim ch3 */

/* PWM ISRs do only sequence-end housekeeping; EasyDMA + NEXTSEQ handle the
 * actual output transitions in hardware, so latency tolerance is high. Use
 * Zephyr's lowest non-zero-latency priority so PWM never preempts TWIM
 * (I2C), SPIM (flash), or BLE LLL — preempting those was the source of
 * stim-on data loss. (NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY = raw NVIC 7
 * exceeds Zephyr's IRQ_PRIO_LOWEST when CONFIG_ZERO_LATENCY_LEVELS=1, so
 * use the Zephyr-side macro for the IRQ_DIRECT_CONNECT priority arg.) */
#define EMS_PWM_IRQ_PRIO     IRQ_PRIO_LOWEST

/* GRTC SYSCOUNTER: 1 MHz → 1 tick = 1 us */
#define MS_TO_TICKS(ms)  ((uint32_t)((ms) * 1000UL))

/* ═══════════════════════════════════════════════════════════════════
 *  Mode Preset Table
 * ═══════════════════════════════════════════════════════════════════ */

/*
 * carrier_on_us = MAX carrier duty at intensity=100.
 * At intensity=1, carrier on-time is always 5us (minimum).
 * Intensity linearly scales between 5us and carrier_on_us.
 */
static const ems_params_t mode_presets[EMS_MODE_COUNT] = {
    [EMS_MODE_SLEEP] = {
        .intensity         = 30,
        .pulse_width_pct   = 90,
        .session_duration_s = 600,  /* 10 minutes default */
        .mode              = EMS_MODE_SLEEP,
        .carrier_period_us = 64,
        .carrier_on_us     = 50,   /* stronger SLEEP (was 30) */
        .burst_duration_ms = 5,
        .burst_interval_ms = 5,    /* halved from 10 → ~2× more bursts/window */
        .rest_duration_ms  = 500,
        .stim_duration_ms  = 1500,
        .inductor_offset_ms = 3,
        .stim_pulses       = 3,
        .stim_period_us    = 1000,
    },
    [EMS_MODE_RELAX] = {
        .intensity         = 50,
        .pulse_width_pct   = 90,
        .session_duration_s = 600,  /* 10 minutes default */
        .mode              = EMS_MODE_RELAX,
        .carrier_period_us = 64,
        .carrier_on_us     = 40,   /* max 40us at intensity=100 (moderate) */
        .burst_duration_ms = 5,
        .burst_interval_ms = 5,
        .rest_duration_ms  = 500,
        .stim_duration_ms  = 1500,
        .inductor_offset_ms = 3,
        .stim_pulses       = 3,
        .stim_period_us    = 1000,
    },
    [EMS_MODE_BREATH] = {
        .intensity         = 50,
        .pulse_width_pct   = 90,
        .session_duration_s = 600,  /* 10 minutes default */
        .mode              = EMS_MODE_BREATH,
        .carrier_period_us = 64,
        .carrier_on_us     = 40,   /* max 40us at intensity=100 (moderate) */
        .burst_duration_ms = 5,
        .burst_interval_ms = 5,
        .rest_duration_ms  = 3000,
        .stim_duration_ms  = 3000,
        .inductor_offset_ms = 3,
        .stim_pulses       = 3,
        .stim_period_us    = 1000,
    },
    [EMS_MODE_CUSTOM] = {  /* If adding modes: update EMS_MODE_COUNT + this table */
        .intensity         = 50,
        .pulse_width_pct   = 90,
        .session_duration_s = 600,  /* 10 minutes default */
        .mode              = EMS_MODE_CUSTOM,
        .carrier_period_us = 64,
        .carrier_on_us     = 45,   /* max 45us at intensity=100 (strong) */
        .burst_duration_ms = 5,
        .burst_interval_ms = 10,
        .rest_duration_ms  = 500,
        .stim_duration_ms  = 1500,
        .inductor_offset_ms = 3,
        .stim_pulses       = 3,
        .stim_period_us    = 1000,
    },
};

/* ═══════════════════════════════════════════════════════════════════
 *  Internal State
 * ═══════════════════════════════════════════════════════════════════ */

BUILD_ASSERT(ARRAY_SIZE(mode_presets) == EMS_MODE_COUNT,
             "mode_presets array size must match EMS_MODE_COUNT");

/* Active + pending parameter sets */
static ems_params_t active_params;
static ems_params_t pending_params;
static atomic_t params_dirty = ATOMIC_INIT(0);

/* Internal parameter store (used by apply_params for hardware value conversion) */
typedef struct {
    long PWM_PERIOD;
    long PWM_ON_TIME;
    long PWM_INTERVAL;
    long PWM_DURATION;
    long CHAN0_PLAYBACK;
    long CHAN0_INDUCTOR;
    long PWM23_DURATION;
    long PWM23_ON_DURATION;
} ems_hw_values_t;

static ems_hw_values_t hw_vals;

/* Per-channel specs (internal, written by apply_params) */
static struct {
    uint64_t active_duration_ns;
    uint64_t interval_ns;
    int8_t   status;
} chan0_spec;

static struct {
    int8_t status;
} chan1_spec;

static struct {
    uint64_t pwm_on_time_ns;
    int8_t   status;
} chan2_spec, chan3_spec;

/* Cached GRTC tick values */
static uint32_t burst_interval_ticks;
static uint32_t inductor_delay_ticks;
static uint32_t rest_duration_ticks;

/* FSM state */
static volatile ems_state_t ems_state = EMS_STATE_IDLE;
static ems_state_t saved_state;  /* for pause/resume */

/* Callbacks + single-cycle mode (used by device FSM) */
static ems_cycle_cb_t cycle_cb;
static ems_cycle_cb_t session_done_cb;
static bool single_cycle_mode;

/* Burst tracking */
static uint16_t power_signal_count;
static uint16_t power_signal_counter;
static atomic_t  active_stimulating_chan = ATOMIC_INIT(2);
static atomic_t chan23_busy = ATOMIC_INIT(0);

/* Session tracking */
static uint32_t session_cycle_count;
static uint32_t session_cycle_limit;

/* (Removed unused globals: is_pwm_modes_enabled, battery_update_ready) */

/* ═══════════════════════════════════════════════════════════════════
 *  PWM Sequences
 * ═══════════════════════════════════════════════════════════════════ */

static const nrfx_pwm_t chan1_pwm_inst = CHAN1_PWM_INST;
static nrf_pwm_values_common_t chan1_seq_common_values = 0;
static nrf_pwm_values_t chan1_seq_values = {.p_common = &chan1_seq_common_values};
static nrf_pwm_sequence_t chan1_sequence = { .length = 1, .repeats = 0, .end_delay = 0 };

/* PWM21 — dedicated to ch2 (P1.11) */
static const nrfx_pwm_t chan2_pwm_inst = CHAN2_PWM_INST;
static nrf_pwm_values_common_t chan2_seq_val;
static nrf_pwm_sequence_t chan2_sequence = {
    .values    = { .p_common = &chan2_seq_val },
    .length    = 1,
    .repeats   = 0,
    .end_delay = 0,
};

/* PWM22 — dedicated to ch3 (P1.07) */
static const nrfx_pwm_t chan3_pwm_inst = CHAN3_PWM_INST;
static nrf_pwm_values_common_t chan3_seq_val;
static nrf_pwm_sequence_t chan3_sequence = {
    .values    = { .p_common = &chan3_seq_val },
    .length    = 1,
    .repeats   = 0,
    .end_delay = 0,
};

/* GRTC compare channels */
static nrfx_grtc_channel_t grtc_cc_a;
static nrfx_grtc_channel_t grtc_cc_b;

/* ═══════════════════════════════════════════════════════════════════
 *  apply_params: ems_params_t → internal hw values
 * ═══════════════════════════════════════════════════════════════════ */

static void apply_params(const ems_params_t *p) {
    /* Write to hw_vals (backward compat) */
    hw_vals.PWM_PERIOD       = p->carrier_period_us;
    hw_vals.PWM_DURATION     = p->burst_duration_ms;
    hw_vals.PWM_INTERVAL     = p->burst_interval_ms;
    hw_vals.CHAN0_INDUCTOR    = p->inductor_offset_ms;
    hw_vals.PWM23_DURATION   = p->stim_period_us;

    /* ── Intensity → mixed: carrier duty + burst count ──
     *
     * Carrier on-time: scales from 5us (intensity=1) to carrier_on_us (intensity=100)
     *   This controls energy per inductor charge cycle.
     *
     * Burst count: scales from 60% to 100% of max bursts (gentle range)
     *   Never drops too low — preserves the 3ms chan23 sync window.
     */
    uint16_t min_on_us = 5;
    uint16_t max_on_us = p->carrier_on_us;
    if (max_on_us <= min_on_us) max_on_us = min_on_us + 1;
    hw_vals.PWM_ON_TIME = min_on_us +
        (uint32_t)(max_on_us - min_on_us) * (p->intensity - 1) / 99;

    uint16_t max_bursts = p->stim_duration_ms / (p->burst_duration_ms + p->burst_interval_ms);
    if (max_bursts < 1) max_bursts = 1;
    /* 60% at intensity=1, 100% at intensity=100 */
    power_signal_count = max_bursts * (60 + (uint32_t)p->intensity * 40 / 100) / 100;
    if (power_signal_count < 1) power_signal_count = 1;

    /* Auto-compute stim_pulses so chan23 ends exactly when chan1 burst ends */
    uint16_t auto_pulses = (uint32_t)p->inductor_offset_ms * 1000 / p->stim_period_us;
    if (auto_pulses < 1) auto_pulses = 1;
    hw_vals.CHAN0_PLAYBACK = auto_pulses;
    hw_vals.PWM23_ON_DURATION = (uint32_t)p->stim_period_us * p->pulse_width_pct / 100;

    /* Chan0 stim/rest durations */
    chan0_spec.active_duration_ns = PWM_MSEC(p->stim_duration_ms);
    chan0_spec.interval_ns        = PWM_MSEC(p->rest_duration_ms);

    /* Chan23 duty cycle */
    chan2_spec.pwm_on_time_ns = PWM_USEC(hw_vals.PWM23_ON_DURATION);
    chan3_spec.pwm_on_time_ns = PWM_USEC(hw_vals.PWM23_ON_DURATION);

    /* Session limit */
    if (p->session_duration_s > 0) {
        uint32_t cycle_ms = p->stim_duration_ms + p->rest_duration_ms;
        session_cycle_limit = (cycle_ms > 0) ? (p->session_duration_s * 1000 / cycle_ms) : 0;
    } else {
        session_cycle_limit = 0; /* infinite */
    }

    /* GRTC tick caches */
    burst_interval_ticks = MS_TO_TICKS(p->burst_interval_ms);
    /* Guard: burst_duration_ms=0 would cause uint16_t underflow (0-1=65535) */
    if (p->burst_duration_ms == 0) {
        inductor_delay_ticks = 0;
    } else {
        uint16_t safe_offset = (p->inductor_offset_ms < p->burst_duration_ms)
                               ? p->inductor_offset_ms : (p->burst_duration_ms - 1);
        inductor_delay_ticks = MS_TO_TICKS(p->burst_duration_ms - safe_offset);
    }
    rest_duration_ticks  = MS_TO_TICKS(p->rest_duration_ms);
}

/* ═══════════════════════════════════════════════════════════════════
 *  PWM Hardware (internal, unchanged logic)
 * ═══════════════════════════════════════════════════════════════════ */

/* Forward declaration — dispatch is called from ISRs */
static void ems_fsm_dispatch(ems_event_t event, uint64_t cc_value);

static void chan1_pwm_event_handler(nrfx_pwm_evt_type_t event_type, void *p_context) {
    if (event_type == NRFX_PWM_EVT_STOPPED) {
        ems_fsm_dispatch(EMS_EVT_BURST_STOPPED, 0);
    }
}

static int chan1_pwm_init(void) {
    nrfx_pwm_config_t conf = NRFX_PWM_DEFAULT_CONFIG(CHAN1_PWM_GPIO_PIN,
                                NRF_PWM_PIN_NOT_CONNECTED,
                                NRF_PWM_PIN_NOT_CONNECTED,
                                NRF_PWM_PIN_NOT_CONNECTED);
    conf.top_value = active_params.carrier_period_us;

    nrfx_pwm_uninit(&chan1_pwm_inst);
    nrfx_err_t ret = nrfx_pwm_init(&chan1_pwm_inst, &conf, chan1_pwm_event_handler, NULL);
    if (ret == NRFX_SUCCESS) {
        IRQ_DIRECT_CONNECT(PWM20_IRQn, EMS_PWM_IRQ_PRIO, nrfx_pwm_20_irq_handler, 0);
        irq_enable(PWM20_IRQn);
        printk("[EMS] PWM20 init OK (top=%d)\n", conf.top_value);
        return 0;
    }
    printk("[EMS] PWM20 init FAIL (0x%08x)\n", ret);
    return -ENODEV;
}

static int chan1_start_burst(void) {
    /* Use intensity-scaled on-time, not raw preset value */
    chan1_seq_common_values = hw_vals.PWM_PERIOD - hw_vals.PWM_ON_TIME;
    chan1_seq_values.p_common = &chan1_seq_common_values;
    chan1_sequence.values = chan1_seq_values;

    uint16_t playback_count = (uint32_t)active_params.burst_duration_ms * 1000
                              / active_params.carrier_period_us;

    if (nrfx_pwm_simple_playback(&chan1_pwm_inst, &chan1_sequence,
                                  playback_count, NRFX_PWM_FLAG_STOP) == 0) {
        chan1_spec.status = 1;
        return 0;
    }
    return -EIO;
}

static void chan2_pwm_event_handler(nrfx_pwm_evt_type_t event_type, void *p_context) {
    if (event_type == NRFX_PWM_EVT_STOPPED) {
        atomic_clear(&chan23_busy);
        atomic_set(&active_stimulating_chan, 3);  /* next burst uses ch3 */
    }
}

static void chan3_pwm_event_handler(nrfx_pwm_evt_type_t event_type, void *p_context) {
    if (event_type == NRFX_PWM_EVT_STOPPED) {
        atomic_clear(&chan23_busy);
        atomic_set(&active_stimulating_chan, 2);  /* next burst uses ch2 */
    }
}

static int chan23_toggle(void) {
    if (atomic_get(&chan23_busy)) return 0;

    uint16_t playback = hw_vals.CHAN0_PLAYBACK;

    if (atomic_get(&active_stimulating_chan) == 2) {
        chan2_seq_val = (uint16_t)(chan2_spec.pwm_on_time_ns / NSEC_PER_USEC);
        atomic_set(&chan23_busy, 1);
        chan2_spec.status = 1;
        nrfx_pwm_simple_playback(&chan2_pwm_inst, &chan2_sequence,
                                  playback, NRFX_PWM_FLAG_STOP);
    } else {
        chan3_seq_val = (uint16_t)(chan3_spec.pwm_on_time_ns / NSEC_PER_USEC);
        atomic_set(&chan23_busy, 1);
        chan3_spec.status = 1;
        nrfx_pwm_simple_playback(&chan3_pwm_inst, &chan3_sequence,
                                  playback, NRFX_PWM_FLAG_STOP);
    }
    return 0;
}

static int chan2_pwm_init(void) {
    nrfx_pwm_config_t conf = {
        .output_pins  = {
            CHAN2_PWM_GPIO_PIN,
            NRF_PWM_PIN_NOT_CONNECTED,
            NRF_PWM_PIN_NOT_CONNECTED,
            NRF_PWM_PIN_NOT_CONNECTED,
        },
        .pin_inverted = { false, false, false, false },
        .irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = active_params.stim_period_us,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO,
    };

    /* Match chan1 symmetry: uninit first so a second acquire after a
     * missed release doesn't trip NRFX_ERROR_ALREADY_INITIALIZED. */
    nrfx_pwm_uninit(&chan2_pwm_inst);
    nrfx_err_t err = nrfx_pwm_init(&chan2_pwm_inst, &conf, chan2_pwm_event_handler, NULL);
    if (err == NRFX_SUCCESS) {
        IRQ_DIRECT_CONNECT(PWM21_IRQn, EMS_PWM_IRQ_PRIO, nrfx_pwm_21_irq_handler, 0);
        irq_enable(PWM21_IRQn);
        printk("[EMS] PWM21 (ch2, P1.11) init OK (top=%d)\n", conf.top_value);
        return 0;
    }
    printk("[EMS] PWM21 init FAIL (0x%08x)\n", err);
    return -EIO;
}

static int chan3_pwm_init(void) {
    nrfx_pwm_config_t conf = {
        .output_pins  = {
            CHAN3_PWM_GPIO_PIN,
            NRF_PWM_PIN_NOT_CONNECTED,
            NRF_PWM_PIN_NOT_CONNECTED,
            NRF_PWM_PIN_NOT_CONNECTED,
        },
        .pin_inverted = { false, false, false, false },
        .irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = active_params.stim_period_us,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO,
    };

    nrfx_pwm_uninit(&chan3_pwm_inst);
    nrfx_err_t err = nrfx_pwm_init(&chan3_pwm_inst, &conf, chan3_pwm_event_handler, NULL);
    if (err == NRFX_SUCCESS) {
        IRQ_DIRECT_CONNECT(PWM22_IRQn, EMS_PWM_IRQ_PRIO, nrfx_pwm_22_irq_handler, 0);
        irq_enable(PWM22_IRQn);
        printk("[EMS] PWM22 (ch3, P1.07) init OK (top=%d)\n", conf.top_value);
        return 0;
    }
    printk("[EMS] PWM22 init FAIL (0x%08x)\n", err);
    return -EIO;
}

/* ═══════════════════════════════════════════════════════════════════
 *  GRTC ISR Wrappers (thin — just translate to FSM events)
 * ═══════════════════════════════════════════════════════════════════ */

static void grtc_cc_a_handler(int32_t id, uint64_t cc_value, void *p_context) {
    ems_fsm_dispatch(EMS_EVT_TIMER_A, cc_value);
}

static void grtc_cc_b_handler(int32_t id, uint64_t cc_value, void *p_context) {
    ems_fsm_dispatch(EMS_EVT_TIMER_B, cc_value);
}

static int grtc_event_init(void) {
    uint8_t ch_a, ch_b;
    nrfx_err_t err = nrfx_grtc_channel_alloc(&ch_a);
    if (err != NRFX_SUCCESS) {
        printk("[EMS] GRTC CC_A alloc FAIL (0x%08x)\n", err);
        return -EIO;
    }
    err = nrfx_grtc_channel_alloc(&ch_b);
    if (err != NRFX_SUCCESS) {
        nrfx_grtc_channel_free(ch_a);
        printk("[EMS] GRTC CC_B alloc FAIL (0x%08x)\n", err);
        return -EIO;
    }
    grtc_cc_a = (nrfx_grtc_channel_t){ .channel = ch_a, .handler = grtc_cc_a_handler };
    grtc_cc_b = (nrfx_grtc_channel_t){ .channel = ch_b, .handler = grtc_cc_b_handler };
    printk("[EMS] GRTC event init OK (CC_A=ch%d, CC_B=ch%d)\n", ch_a, ch_b);
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════
 *  Central FSM Dispatch — ALL transitions in one place
 *
 *  ┌──────────────────────────────────────────────────────────┐
 *  │ State         │ Event          │ Next State              │
 *  ├───────────────┼────────────────┼─────────────────────────┤
 *  │ IDLE          │ ENABLE         │ BURST_GAP               │
 *  │ BURST_GAP     │ TIMER_A        │ BURST_ACTIVE            │
 *  │ BURST_ACTIVE  │ TIMER_B        │ BURST_ACTIVE            │
 *  │ BURST_ACTIVE  │ BURST_STOPPED  │ BURST_GAP / REST / IDLE │
 *  │ REST          │ TIMER_A        │ BURST_ACTIVE            │
 *  │ PAUSED        │ RESUME         │ (saved_state)           │
 *  │ *any*         │ DISABLE        │ IDLE                    │
 *  │ *any running* │ PAUSE          │ PAUSED                  │
 *  └──────────────────────────────────────────────────────────┘
 * ═══════════════════════════════════════════════════════════════════ */

static void ems_fsm_dispatch(ems_event_t event, uint64_t cc_value) {

    /* ── Global: DISABLE from any state ── */
    if (event == EMS_EVT_DISABLE) {
        nrfx_grtc_syscounter_cc_int_disable(grtc_cc_a.channel);
        nrfx_grtc_syscounter_cc_int_disable(grtc_cc_b.channel);
        nrfx_pwm_stop(&chan1_pwm_inst, false);
        nrfx_pwm_stop(&chan2_pwm_inst, false);
        nrfx_pwm_stop(&chan3_pwm_inst, false);
        atomic_clear(&chan23_busy);
        nrf_gpio_pin_set(CHAN0_GPIO_PIN);
        chan0_spec.status = 1;
        chan1_spec.status = 0;
        single_cycle_mode = false; /* FIX #6: clear on any disable */
        ems_state = EMS_STATE_IDLE;
        printk("[EMS] FSM → IDLE (disabled)\n");
        return;
    }

    /* ── Global: PAUSE from any running state ── */
    if (event == EMS_EVT_PAUSE &&
        ems_state != EMS_STATE_IDLE && ems_state != EMS_STATE_PAUSED) {
        saved_state = ems_state;
        nrfx_grtc_syscounter_cc_int_disable(grtc_cc_a.channel);
        nrfx_grtc_syscounter_cc_int_disable(grtc_cc_b.channel);
        nrfx_pwm_stop(&chan1_pwm_inst, false);
        nrfx_pwm_stop(&chan2_pwm_inst, false);
        nrfx_pwm_stop(&chan3_pwm_inst, false);
        atomic_clear(&chan23_busy);
        ems_state = EMS_STATE_PAUSED;
        printk("[EMS] FSM → PAUSED (from %d)\n", saved_state);
        return;
    }

    /* ── Per-state transitions ── */
    switch (ems_state) {

    /* ─────── IDLE ─────── */
    case EMS_STATE_IDLE:
        if (event == EMS_EVT_ENABLE) {
            /* Apply pending params if dirty, else use active.
             * IRQ lock ensures struct copy is atomic vs BLE thread writes. */
            if (atomic_get(&params_dirty)) {
                unsigned int key = irq_lock();
                active_params = pending_params;
                atomic_clear(&params_dirty);
                irq_unlock(key);
                apply_params(&active_params);
            }
            power_signal_counter = 0;
            session_cycle_count = 0;
            nrf_gpio_pin_clear(CHAN0_GPIO_PIN);
            chan0_spec.status = 0;
            ems_state = EMS_STATE_BURST_GAP;
            nrfx_grtc_syscounter_cc_relative_set(&grtc_cc_a, 10,
                                                 true, NRFX_GRTC_CC_RELATIVE_SYSCOUNTER);
            printk("[EMS] FSM → BURST_GAP (enabled, mode=%d, intensity=%d)\n",
                   active_params.mode, active_params.intensity);
        }
        break;

    /* ─────── BURST_GAP ─────── */
    case EMS_STATE_BURST_GAP:
        if (event == EMS_EVT_TIMER_A) {
            chan1_start_burst();
            power_signal_counter++;
            ems_state = EMS_STATE_BURST_ACTIVE;
            nrfx_grtc_syscounter_cc_absolute_set(&grtc_cc_b,
                                                 cc_value + inductor_delay_ticks, true);
        }
        break;

    /* ─────── BURST_ACTIVE ─────── */
    case EMS_STATE_BURST_ACTIVE:
        if (event == EMS_EVT_TIMER_B) {
            /* Inductor delay reached → trigger chan23 stimulation */
            chan23_toggle();

        } else if (event == EMS_EVT_BURST_STOPPED) {
            chan1_spec.status = 0;

            if (power_signal_counter >= power_signal_count) {
                /* Last burst → notify cycle complete */
                session_cycle_count++;

                /* Invoke cycle callback (device FSM uses this) */
                if (cycle_cb) cycle_cb();

                if (session_cycle_limit > 0 && session_cycle_count >= session_cycle_limit) {
                    printk("[EMS] Session complete (%u cycles)\n", session_cycle_count);
                    /* Notify device FSM BEFORE disabling */
                    if (session_done_cb) session_done_cb();
                    ems_fsm_dispatch(EMS_EVT_DISABLE, 0);
                    return;
                }
                /* Enter rest phase */
                nrf_gpio_pin_set(CHAN0_GPIO_PIN);
                chan0_spec.status = 1;
                ems_state = EMS_STATE_REST;
                nrfx_grtc_syscounter_cc_relative_set(&grtc_cc_a, rest_duration_ticks,
                                                     true, NRFX_GRTC_CC_RELATIVE_SYSCOUNTER);
            } else {
                /* More bursts → interval gap */
                ems_state = EMS_STATE_BURST_GAP;
                nrfx_grtc_syscounter_cc_relative_set(&grtc_cc_a, burst_interval_ticks,
                                                     true, NRFX_GRTC_CC_RELATIVE_SYSCOUNTER);
            }
        }
        break;

    /* ─────── REST ─────── */
    case EMS_STATE_REST:
        if (event == EMS_EVT_TIMER_A) {
            /* One full stim+rest cycle complete → notify device FSM */
            if (cycle_cb) {
                cycle_cb();
            }

            /* Single-cycle mode: stop after one complete cycle */
            if (single_cycle_mode) {
                single_cycle_mode = false;
                ems_fsm_dispatch(EMS_EVT_DISABLE, 0);
                return;
            }

            /* Rest ended → apply pending params, start new stim phase */
            if (atomic_get(&params_dirty)) {
                /* Save old periods to detect if PWM reinit is needed */
                uint16_t old_carrier = active_params.carrier_period_us;
                uint16_t old_stim    = active_params.stim_period_us;
                {
                    unsigned int key = irq_lock();
                    active_params = pending_params;
                    atomic_clear(&params_dirty);
                    irq_unlock(key);
                }
                apply_params(&active_params);
                /* Only reinit PWM hardware if period (top_value) changed */
                if (active_params.carrier_period_us != old_carrier) {
                    chan1_pwm_init();
                }
                if (active_params.stim_period_us != old_stim) {
                    nrfx_pwm_config_t stim_conf = {
                        .output_pins  = {
                            CHAN2_PWM_GPIO_PIN,
                            NRF_PWM_PIN_NOT_CONNECTED,
                            NRF_PWM_PIN_NOT_CONNECTED,
                            NRF_PWM_PIN_NOT_CONNECTED,
                        },
                        .pin_inverted = { false, false, false, false },
                        .irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
                        .base_clock   = NRF_PWM_CLK_1MHz,
                        .count_mode   = NRF_PWM_MODE_UP,
                        .top_value    = active_params.stim_period_us,
                        .load_mode    = NRF_PWM_LOAD_COMMON,
                        .step_mode    = NRF_PWM_STEP_AUTO,
                    };
                    nrfx_pwm_reconfigure(&chan2_pwm_inst, &stim_conf);
                    stim_conf.output_pins[0] = CHAN3_PWM_GPIO_PIN;
                    nrfx_pwm_reconfigure(&chan3_pwm_inst, &stim_conf);
                }
            }
            nrf_gpio_pin_clear(CHAN0_GPIO_PIN);
            chan0_spec.status = 0;
            power_signal_counter = 0;

            /* Start first burst immediately */
            chan1_start_burst();
            power_signal_counter++;
            ems_state = EMS_STATE_BURST_ACTIVE;
            nrfx_grtc_syscounter_cc_absolute_set(&grtc_cc_b,
                                                 cc_value + inductor_delay_ticks, true);
        }
        break;

    /* ─────── PAUSED ─────── */
    case EMS_STATE_PAUSED:
        if (event == EMS_EVT_RESUME) {
            /* Apply any pending params */
            if (atomic_get(&params_dirty)) {
                active_params = pending_params;
                atomic_clear(&params_dirty);
                apply_params(&active_params);
            }
            /* Resume from BURST_GAP (simplest safe re-entry) */
            power_signal_counter = 0;
            nrf_gpio_pin_clear(CHAN0_GPIO_PIN);
            chan0_spec.status = 0;
            ems_state = EMS_STATE_BURST_GAP;
            nrfx_grtc_syscounter_cc_relative_set(&grtc_cc_a, 10,
                                                 true, NRFX_GRTC_CC_RELATIVE_SYSCOUNTER);
            printk("[EMS] FSM → BURST_GAP (resumed)\n");
        }
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════ */

/* Acquire all 3 PWM peripherals — call before starting stim. Idempotent:
 * each chanN_pwm_init() uninits first, so re-acquire after a missed release
 * is safe. On partial failure, roll back already-initialized channels so
 * PWM pins return to HiZ and the caller sees a clean all-or-nothing result. */
static int ems_pwm_acquire(void)
{
    int ret = chan1_pwm_init();
    if (ret) return ret;
    ret = chan2_pwm_init();
    if (ret) {
        nrfx_pwm_uninit(&chan1_pwm_inst);
        return ret;
    }
    ret = chan3_pwm_init();
    if (ret) {
        nrfx_pwm_uninit(&chan2_pwm_inst);
        nrfx_pwm_uninit(&chan1_pwm_inst);
        return ret;
    }
    return 0;
}

/* Release all 3 PWM peripherals — frees pins back to HiZ inputs and gates
 * the PWM peripheral clocks. Saves ~1-2 mA when EMS is idle (the PWM pins
 * are connected to gate drivers that sink current when driven low). */
static void ems_pwm_release(void)
{
    nrfx_pwm_uninit(&chan1_pwm_inst);
    nrfx_pwm_uninit(&chan2_pwm_inst);
    nrfx_pwm_uninit(&chan3_pwm_inst);
}

/* P0.0 is wired to a discharge path for the stim-circuit inductor: holding
 * it HIGH lets residual charge bleed out via the snubber resistor, LOW
 * disconnects the discharge (idle, low-power). It must be HIGH:
 *   1. For 60 s after boot (in case of warm reset mid-stim → residual L)
 *   2. Continuously during a stim session (existing FSM behavior)
 *   3. For 60 s after stim ends (drain inductor before sleep)
 * Otherwise it stays LOW to save power. */
/* Boot-time grace: after warm reset the inductor may hold residual energy.
 * Drive P0.0 HIGH for this long after init to drain it. */
#define DISCHARGE_HOLD_S        60
/* Post-stop bleed: after a normal session ends and LS1 is cut, hold P0.0
 * HIGH briefly so the snubber drains the inductor with no rail backfeed. */
#define DISCHARGE_POST_STOP_MS  500

extern struct k_work_q ems_wq;

static void discharge_off_work_fn(struct k_work *w)
{
    (void)w;
    /* Only release if EMS is still idle. If a session started during the
     * 60 s grace, leave P0.0 alone (FSM owns it during active stim). */
    if (ems_state == EMS_STATE_IDLE) {
        nrf_gpio_pin_clear(CHAN0_GPIO_PIN);
        printk("[EMS] discharge hold expired → P0.0 LOW\n");
    }
}
static K_WORK_DELAYABLE_DEFINE(discharge_off_work, discharge_off_work_fn);

/* Schedule (or re-schedule) P0.0 to drop LOW after DISCHARGE_HOLD_S.
 * Drives P0.0 HIGH immediately. Idempotent. */
static void ems_discharge_hold_start(void)
{
    nrf_gpio_pin_set(CHAN0_GPIO_PIN);
    /* Cancel any pending instance, then schedule fresh on ems_wq. We use
     * k_work_schedule_for_queue (not _reschedule) so the work item gets
     * bound to ems_wq on first call — _reschedule on a never-submitted
     * delayable work item silently fails to bind to a queue. */
    k_work_cancel_delayable(&discharge_off_work);
    int rc = k_work_schedule_for_queue(&ems_wq, &discharge_off_work,
                                       K_SECONDS(DISCHARGE_HOLD_S));
    printk("[EMS] discharge hold scheduled (%d s) rc=%d\n",
           DISCHARGE_HOLD_S, rc);
}

int ems_init(void) {
    int ret;

    printk("[EMS] === Init start ===\n");

    /* Default params */
    active_params = mode_presets[EMS_MODE_SLEEP];
    pending_params = active_params;
    apply_params(&active_params);

    /* P0.0 — discharge bleed for stim inductor.
     * Configure as output, drive HIGH for the boot grace window, then
     * the work item drops it LOW after DISCHARGE_HOLD_S.
     *
     * NOTE: dppi_safety_init() in deep_sleep.c re-configures this pin
     * via the Zephyr GPIO driver after ems_init() runs. Direct nrf_gpio
     * calls bypass the driver safely (register writes). */
    nrf_gpio_cfg_output(CHAN0_GPIO_PIN);
    ems_discharge_hold_start();

    /* PWMs are NOT initialized at boot — they would drive the gate driver
     * pins and waste ~1-2 mA continuously. ems_start() acquires them on
     * demand and ems_stop() releases them. */

    /* GRTC compare channels */
    ret = grtc_event_init();
    if (ret) return ret;

    printk("[EMS] === Init complete (mode=SLEEP, intensity=%d, pw=%d%%) ===\n",
           active_params.intensity, active_params.pulse_width_pct);
    return 0;
}

int ems_start(void) {
    if (ems_state != EMS_STATE_IDLE) return -EALREADY;
    /* Acquire PWM peripherals on demand — they drive the gate driver
     * pins and waste current when held active outside of stim sessions. */
    int ret = ems_pwm_acquire();
    if (ret) {
        printk("[EMS] PWM acquire failed: %d\n", ret);
        return ret;
    }
    ems_fsm_dispatch(EMS_EVT_ENABLE, 0);
    return 0;
}

void ems_stop(void) {
    if (ems_state == EMS_STATE_IDLE) return;
    ems_fsm_dispatch(EMS_EVT_DISABLE, 0);
    /* Release PWM peripherals so the pins return to HiZ inputs and the
     * PWM block clocks gate. Saves ~1-2 mA between sessions. */
    ems_pwm_release();
    /* Leave P0.0 LOW here. ems_device_fsm.stop_all() cuts LS1 next, then
     * calls ems_discharge_after_stop() to drive the bleed without the
     * rail backfeeding the inductor. */
    k_work_cancel_delayable(&discharge_off_work);
    nrf_gpio_pin_clear(CHAN0_GPIO_PIN);
    printk("[EMS] stop → P0.0 LOW (rail teardown next)\n");
}

void ems_discharge_after_stop(void) {
    /* Caller has already cut LS1. Drive P0.0 HIGH so residual inductor
     * energy drains through the snubber, then drop it after a short hold. */
    nrf_gpio_pin_set(CHAN0_GPIO_PIN);
    k_work_cancel_delayable(&discharge_off_work);
    int rc = k_work_schedule_for_queue(&ems_wq, &discharge_off_work,
                                       K_MSEC(DISCHARGE_POST_STOP_MS));
    printk("[EMS] post-stop bleed: P0.0 HIGH for %d ms (rc=%d)\n",
           DISCHARGE_POST_STOP_MS, rc);
}

void ems_pause(void) {
    ems_fsm_dispatch(EMS_EVT_PAUSE, 0);
}

void ems_resume(void) {
    ems_fsm_dispatch(EMS_EVT_RESUME, 0);
}

/*
 * FIX #23: Parameter setters MERGE into pending_params instead of overwriting.
 * This prevents mode→intensity or intensity→pulse_width sequences from
 * losing the first change.
 */

void ems_set_mode(ems_mode_t mode) {
    if (mode >= EMS_MODE_COUNT) return;
    /* Mode overrides all timing params but preserves session_duration */
    uint32_t saved_session = pending_params.session_duration_s;
    pending_params = mode_presets[mode];
    pending_params.session_duration_s = saved_session;
    atomic_set(&params_dirty, 1);
    printk("[EMS] Mode → %d (pending)\n", mode);

    if (ems_state == EMS_STATE_IDLE) {
        active_params = pending_params;
        atomic_clear(&params_dirty);
        apply_params(&active_params);
    }
}

void ems_set_intensity(uint8_t intensity) {
    if (intensity < 1) intensity = 1;
    if (intensity > 100) intensity = 100;
    /* Merge: only update intensity field, keep everything else in pending */
    pending_params.intensity = intensity;
    pending_params.mode = EMS_MODE_CUSTOM;
    atomic_set(&params_dirty, 1);
    printk("[EMS] Intensity → %d (pending)\n", intensity);

    if (ems_state == EMS_STATE_IDLE) {
        active_params = pending_params;
        atomic_clear(&params_dirty);
        apply_params(&active_params);
    }
}

void ems_set_pulse_width(uint8_t pct) {
    if (pct < 1) pct = 1;
    if (pct > 100) pct = 100;
    /* Merge: only update pulse_width field */
    pending_params.pulse_width_pct = pct;
    pending_params.mode = EMS_MODE_CUSTOM;
    atomic_set(&params_dirty, 1);
    printk("[EMS] Pulse width → %d%% (pending)\n", pct);

    if (ems_state == EMS_STATE_IDLE) {
        active_params = pending_params;
        atomic_clear(&params_dirty);
        apply_params(&active_params);
    }
}

void ems_set_session_duration(uint32_t secs) {
    pending_params.session_duration_s = secs;
    active_params.session_duration_s = secs;
    /* Recompute limit immediately */
    if (secs > 0) {
        uint32_t cycle_ms = active_params.stim_duration_ms + active_params.rest_duration_ms;
        session_cycle_limit = (cycle_ms > 0) ? (secs * 1000 / cycle_ms) : 0;
    } else {
        session_cycle_limit = 0;
    }
    printk("[EMS] Session duration → %u s (limit=%u cycles)\n", secs, session_cycle_limit);
}

void ems_apply_params_direct(const ems_params_t *params) {
    pending_params = *params;
    atomic_set(&params_dirty, 1);
    printk("[EMS] Params direct (mode=%d, intensity=%d, pw=%d%%)\n",
           params->mode, params->intensity, params->pulse_width_pct);

    if (ems_state == EMS_STATE_IDLE) {
        active_params = pending_params;
        atomic_clear(&params_dirty);
        apply_params(&active_params);
    }
}

ems_state_t ems_get_state(void) {
    return ems_state;
}

ems_params_t ems_get_params(void) {
    return active_params;
}

void ems_register_cycle_cb(ems_cycle_cb_t cb) {
    cycle_cb = cb;
}

void ems_register_session_done_cb(ems_cycle_cb_t cb) {
    session_done_cb = cb;
}

void ems_set_single_cycle(bool enable) {
    single_cycle_mode = enable;
}
