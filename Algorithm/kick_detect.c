/**
 ******************************************************************************
 * @file    kick_detect.c
 * @brief   Freestyle kick detection algorithm
 *
 * State machine overview:
 *
 *   [STATE_INIT]
 *       │
 *       ├─ Check |pitch| < 15° and |roll| < 15°
 *       │
 *       ├─ FAIL: Red LED + Long Beep + Motor → stay in INIT
 *       │
 *       └─ PASS: Green LED + Short Beep → STATE_POSTURE_OK
 *
 *   [STATE_POSTURE_OK]
 *       │
 *       └─ Transition immediately to STATE_DETECT (capture baseline)
 *
 *   [STATE_DETECT]
 *       │
 *       ├─ Monitor acc_z peaks within 2s window
 *       ├─ Detect kick cycles from peak-trough patterns
 *       ├─ Estimate displacement via (acc_pp * t² / π²) formula
 *       ├─ Check roll change and lateral displacement
 *       │
 *       ├─ amplitude ≥ 30cm + standard form → Green LED + Short Beep
 *       ├─ amplitude ≥ 30cm + bad form           → Yellow LED + Long Beep + Motor
 *       ├─ amplitude < 30cm                      → Red LED
 *       │
 *       └─ 5s timeout → return to STATE_INIT
 *
 *   Key press (via KickDetect_Reset) at any point → STATE_INIT
 *
 * @note    Feedback calls (LED/beep/motor) are blocking and run at state
 *          transitions only. During continuous detection (while loop),
 *          the system samples IMU data without blocking.
 ******************************************************************************
 */

#include "kick_detect.h"
#include "bsp_imu.h"
#include "bsp_beep.h"
#include "bsp_motor.h"
#include "bsp_ws2812.h"

/* -------------------------------------------------------------------------- */
/*  Internal state                                                            */
/* -------------------------------------------------------------------------- */

static KickState_t  kick_state  = STATE_INIT;
static KickResult_t kick_result = KICK_RESULT_NONE;
static uint32_t     state_entry_tick = 0;
static uint32_t     last_eval_tick   = 0;

/* Posture check — baseline orientation captured on transition to DETECT */
static float baseline_roll  = 0.0f;
static float baseline_pitch = 0.0f;

/* Kick cycle peak tracking */
static float    acc_z_min       = 0.0f;
static float    acc_z_max       = 0.0f;
static float    roll_min        = 0.0f;
static float    roll_max        = 0.0f;
static float    acc_x_drift     = 0.0f;     /* Accumulated horizontal acc integration */
static float    acc_y_drift     = 0.0f;
static uint32_t cycle_start_tick = 0;
static uint8_t  cycle_active    = 0;
static uint8_t  cycle_has_peak  = 0;
static uint8_t  cycle_has_trough = 0;

/* Simplified velocity for zero-crossing detection */
static float    vel_z_est       = 0.0f;
static float    acc_z_prev      = 0.0f;
static uint32_t last_sample_tick = 0;

/* Vertical acceleration baseline (gravity component, ~9.8 m/s² when horizontal) */
#define GRAVITY_MSS  9.8f

/* Detection threshold for acc_z deviation from gravity (start of kick cycle) */
#define KICK_START_THRESHOLD_MSS   3.0f

/* -------------------------------------------------------------------------- */
/*  Internal helpers                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Apply feedback and transition to new state
 */
static void kick_set_state(KickState_t new_state)
{
    kick_state = new_state;
    state_entry_tick = HAL_GetTick();
}

/**
 * @brief  Estimate vertical displacement from acceleration peak-to-peak
 *         and half-cycle duration, using sinusoidal motion model:
 *
 *           disp_cm = (acc_pp * half_period_s² / π²) * 100
 *
 * @param  acc_pp      Peak-to-peak vertical acceleration (m/s²)
 * @param  duration_ms Duration of the detected cycle (ms)
 * @return Estimated peak-to-peak displacement in cm
 */
static float estimate_displacement_cm(float acc_pp, uint32_t duration_ms)
{
    if (duration_ms == 0) return 0.0f;

    float half_period_s = (float)duration_ms / 2000.0f;  /* /2 for half-period, /1000 for ms→s */
    float pi = 3.14159265f;
    float disp_m = (acc_pp * half_period_s * half_period_s) / (pi * pi);
    return disp_m * 100.0f;
}

/**
 * @brief  Reset kick cycle tracking variables
 */
static void kick_cycle_reset(void)
{
    acc_z_min      = 0.0f;
    acc_z_max      = 0.0f;
    roll_min       = 0.0f;
    roll_max       = 0.0f;
    acc_x_drift    = 0.0f;
    acc_y_drift    = 0.0f;
    cycle_start_tick = 0;
    cycle_active   = 0;
    cycle_has_peak = 0;
    cycle_has_trough = 0;
    vel_z_est      = 0.0f;
    acc_z_prev     = 0.0f;
    last_sample_tick = 0;
}

/**
 * @brief  Evaluate a completed kick cycle and set kick_result
 */
static void kick_cycle_evaluate(void)
{
    float acc_pp   = acc_z_max - acc_z_min;           /* Peak-to-peak acc (m/s²) */
    float roll_chg = roll_max - roll_min;              /* Roll change (deg) */

    if (roll_chg < 0.0f) roll_chg = -roll_chg;

    /* Estimate vertical displacement */
    uint32_t duration_ms = HAL_GetTick() - cycle_start_tick;
    float amplitude_cm = estimate_displacement_cm(acc_pp, duration_ms);

    /* Estimate horizontal displacement (simplified integration of x/y acc) */
    float horiz_disp = 0.0f;
    if (acc_x_drift < 0.0f) acc_x_drift = -acc_x_drift;
    if (acc_y_drift < 0.0f) acc_y_drift = -acc_y_drift;
    horiz_disp = acc_x_drift + acc_y_drift;

    /* Classify the kick */
    if (amplitude_cm < KICK_AMPLITUDE_THRESHOLD_CM)
    {
        /* Insufficient amplitude */
        kick_result = KICK_RESULT_BAD_AMPLITUDE;
        BSP_WS2812_Red();
    }
    else if (roll_chg > FLIP_ROLL_THRESHOLD_DEG ||
             horiz_disp > HORIZONTAL_DISPLACEMENT_THRESHOLD_M)
    {
        /* Good amplitude but bad form (flip or lateral drift) */
        kick_result = KICK_RESULT_BAD_FORM;
        BSP_WS2812_Yellow();
        BSP_Beep_Long();
        BSP_Motor_Vibrate();
    }
    else
    {
        /* Good kick! */
        kick_result = KICK_RESULT_GOOD;
        BSP_WS2812_Green();
        BSP_Beep_Short();
    }

    last_eval_tick = HAL_GetTick();
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize kick detection state machine
 */
void KickDetect_Init(void)
{
    kick_state  = STATE_INIT;
    kick_result = KICK_RESULT_NONE;
    kick_cycle_reset();
    state_entry_tick = HAL_GetTick();
    last_eval_tick   = 0;
}

/**
 * @brief  Main processing loop — call from main while(1) as fast as possible
 *
 * Reads latest IMU data and runs the state machine. Non-blocking except
 * during state-transition feedback (beeps/vibration).
 */
void KickDetect_Process(void)
{
    /* Skip if no new IMU data */
    if (!BSP_IMU_DataReady()) return;

    IMU_Data_t imu;
    BSP_IMU_GetData(&imu);
    BSP_IMU_ClearDataReady();

    /* ---- STATE_INIT: check horizontal posture ---- */
    if (kick_state == STATE_INIT)
    {
        float p = imu.pitch;
        float r = imu.roll;

        /* Make angles positive for threshold comparison */
        if (p < 0.0f) p = -p;
        if (r < 0.0f) r = -r;

        if (p <= POSTURE_HORIZONTAL_THRESHOLD_DEG &&
            r <= POSTURE_HORIZONTAL_THRESHOLD_DEG)
        {
            /* Posture OK — feedback and transition */
            BSP_WS2812_Green();
            BSP_Beep_Short();

            /* Capture baseline orientation */
            baseline_roll  = imu.roll;
            baseline_pitch = imu.pitch;

            kick_set_state(STATE_POSTURE_OK);
        }
        else
        {
            /* Not horizontal — warn user */
            BSP_WS2812_Red();
            BSP_Beep_Long();
            BSP_Motor_Vibrate();
            /* Stay in STATE_INIT */
        }

        kick_result = KICK_RESULT_NONE;
        return;
    }

    /* ---- STATE_POSTURE_OK: transition to DETECT ---- */
    if (kick_state == STATE_POSTURE_OK)
    {
        kick_cycle_reset();
        kick_set_state(STATE_DETECT);
        /* Fall through to STATE_DETECT processing */
    }

    /* ---- STATE_DETECT: monitor kick motion ---- */

    /* Check timeout */
    if ((HAL_GetTick() - state_entry_tick) > KICK_DETECT_TIMEOUT_MS)
    {
        /* No activity for too long — go back to posture check */
        BSP_WS2812_Off();
        kick_set_state(STATE_INIT);
        kick_result = KICK_RESULT_NONE;
        return;
    }

    /* Cooldown: don't evaluate immediately after a previous evaluation */
    if (last_eval_tick != 0 &&
        (HAL_GetTick() - last_eval_tick) < KICK_EVAL_COOLDOWN_MS)
    {
        return;
    }

    /* ---- Kick cycle detection on vertical acceleration ---- */
    float acc_z_motion = imu.acc_z - GRAVITY_MSS;

    /* Start of kick cycle: acceleration deviates from gravity */
    if (!cycle_active)
    {
        float abs_acc = acc_z_motion;
        if (abs_acc < 0.0f) abs_acc = -abs_acc;

        if (abs_acc > KICK_START_THRESHOLD_MSS)
        {
            /* Kick cycle started */
            cycle_active    = 1;
            cycle_start_tick = HAL_GetTick();
            acc_z_min       = acc_z_motion;
            acc_z_max       = acc_z_motion;
            roll_min        = imu.roll;
            roll_max        = imu.roll;
            acc_x_drift     = 0.0f;
            acc_y_drift     = 0.0f;
            cycle_has_peak  = 0;
            cycle_has_trough = 0;
            vel_z_est       = 0.0f;
            acc_z_prev      = acc_z_motion;
            last_sample_tick = imu.timestamp;
        }
        else
        {
            /* Not kicking — reset result for LED feedback */
            if (kick_result != KICK_RESULT_NONE &&
                (HAL_GetTick() - last_eval_tick) > 1000U)
            {
                kick_result = KICK_RESULT_NONE;
                BSP_WS2812_Off();
            }
        }
        return;
    }

    /* ---- Active cycle: accumulate peaks and track orientation ---- */

    /* Update acc_z peaks */
    if (acc_z_motion > acc_z_max) acc_z_max = acc_z_motion;
    if (acc_z_motion < acc_z_min) acc_z_min = acc_z_motion;

    /* Track roll envelope */
    if (imu.roll > roll_max) roll_max = imu.roll;
    if (imu.roll < roll_min) roll_min = imu.roll;

    /* Accumulate horizontal acceleration (simplified drift proxy) */
    {
        float dt = 0.01f;  /* ~100Hz default */
        if (last_sample_tick != 0 && imu.timestamp != last_sample_tick)
        {
            dt = (float)(imu.timestamp - last_sample_tick) / 1000.0f;
            if (dt > 0.5f) dt = 0.01f;  /* Sanity clamp */
        }
        acc_x_drift += imu.acc_x * dt;
        acc_y_drift += imu.acc_y * dt;
    }
    last_sample_tick = imu.timestamp;

    /* Detect peak/trough transitions via velocity zero-crossing */
    if (cycle_active)
    {
        /* Simple velocity estimation (leaky integrator) */
        float dt = 0.01f;
        float vel_new = vel_z_est + acc_z_motion * dt;

        /* Detect zero-crossing of velocity (peak reached) */
        if (vel_z_est > 0.0f && vel_new <= 0.0f)
        {
            cycle_has_peak = 1;
        }
        if (vel_z_est < 0.0f && vel_new >= 0.0f)
        {
            cycle_has_trough = 1;
        }
        vel_z_est = vel_new;
    }

    /* Check if cycle is complete (both peak and trough detected) */
    if (cycle_active && cycle_has_peak && cycle_has_trough)
    {
        kick_cycle_evaluate();
        kick_cycle_reset();
        return;
    }

    /* Safety: if cycle runs too long without completion, reset */
    if (cycle_active &&
        (HAL_GetTick() - cycle_start_tick) > SAMPLE_WINDOW_MS)
    {
        /* Force evaluate whatever we have */
        if (acc_z_max - acc_z_min > KICK_START_THRESHOLD_MSS)
        {
            kick_cycle_evaluate();
        }
        kick_cycle_reset();
    }
}

/**
 * @brief  Get current state machine state
 */
KickState_t KickDetect_GetState(void)
{
    return kick_state;
}

/**
 * @brief  Get last kick evaluation result
 */
KickResult_t KickDetect_GetResult(void)
{
    return kick_result;
}

/**
 * @brief  Reset state machine to INIT (e.g., on key press)
 */
void KickDetect_Reset(void)
{
    kick_set_state(STATE_INIT);
    kick_result = KICK_RESULT_NONE;
    kick_cycle_reset();
    BSP_WS2812_Off();
    BSP_Beep_Off();
    BSP_Motor_Off();
}
