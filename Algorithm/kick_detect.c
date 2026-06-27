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
//#include "bsp_motor.h"
#include "bsp_ws2812.h"

/* -------------------------------------------------------------------------- */
/*  Internal state                                                            */
/* -------------------------------------------------------------------------- */

KickDebug_t kick_dbg;

static KickState_t  kick_state  = STATE_IDLE;
static KickResult_t kick_result = KICK_RESULT_NONE;
static uint32_t     state_entry_tick = 0;
static uint32_t     last_eval_tick   = 0;
static uint8_t      init_checked = 0;

static float baseline_roll  = 0.0f;
static float baseline_pitch = 0.0f;

/* Kick cycle peak tracking (调试可见) */
float    acc_z_min, acc_z_max;
float    roll_min,  roll_max;
uint32_t cycle_start_tick;
uint8_t  cycle_active, cycle_has_peak, cycle_has_trough;

/* 三轴积分状态（调试可见） */
float    vel_x,  vel_y,  vel_z;
float    pos_x,  pos_y,  pos_z;
float    pos_z_max, pos_z_min;       /* 周期内位置峰谷 */
uint32_t last_sample_tick;

/* Detection threshold for acc_z deviation from baseline (start of kick cycle) */
#define KICK_START_THRESHOLD_MSS   3.0f

/* -------------------------------------------------------------------------- */
/*  Internal helpers                                                          */
/* -------------------------------------------------------------------------- */

static void kick_set_state(KickState_t new_state)
{
    kick_state = new_state;
    state_entry_tick = HAL_GetTick();
}

/**
 * @brief  Reset kick cycle tracking variables
 */
static void kick_cycle_reset(void)
{
    acc_z_min  = 0.0f;  acc_z_max  = 0.0f;
    roll_min   = 0.0f;  roll_max   = 0.0f;
    vel_x = 0.0f;  vel_y = 0.0f;  vel_z = 0.0f;
    pos_x = 0.0f;  pos_y = 0.0f;  pos_z = 0.0f;
    pos_z_max = 0.0f;  pos_z_min = 0.0f;
    cycle_start_tick = 0;
    cycle_active = 0;
    cycle_has_peak = 0;
    cycle_has_trough = 0;
    last_sample_tick = 0;
}

/**
 * @brief  Evaluate a completed kick cycle and set kick_result
 */
static void kick_cycle_evaluate(void)
{
    float acc_pp   = acc_z_max - acc_z_min;
    float roll_chg = roll_max - roll_min;

    if (roll_chg < 0.0f) roll_chg = -roll_chg;

    /* 积分得到的位移 */
    float amplitude_cm = pos_z_max - pos_z_min;
    float horiz_disp   = (pos_x > 0 ? pos_x : -pos_x) + (pos_y > 0 ? pos_y : -pos_y);

    /* Update debug metrics */
    kick_dbg.force    = acc_pp;
    kick_dbg.roll_chg = roll_chg;
    kick_dbg.disp_z   = amplitude_cm;
    kick_dbg.disp_x   = pos_x;
    kick_dbg.disp_y   = pos_y;

    /* Classify the kick */
    if (amplitude_cm < KICK_AMPLITUDE_MIN_CM)
    {
        return;
    }
    if (amplitude_cm < KICK_AMPLITUDE_THRESHOLD_CM)
    {
        /* 30~50cm: 幅度不足 */
        kick_result = KICK_RESULT_BAD_AMPLITUDE;
        BSP_WS2812_Yellow();
        BSP_Beep_Short();
    }
    else if (horiz_disp > HORIZONTAL_DISPLACEMENT_THRESHOLD_CM)
    {
        kick_result = KICK_RESULT_BAD_AMPLITUDE;
        BSP_WS2812_Red();
        BSP_Beep_Long();
    }
    else if (acc_pp >= KICK_FORCE_WEAK_THRESHOLD)
    {
        kick_result = KICK_RESULT_GOOD;
        kick_dbg.kick_cnt++;
        BSP_WS2812_Green();
        BSP_Beep_Short();
    }
    else
    {
        kick_result = KICK_RESULT_BAD_AMPLITUDE;
        BSP_WS2812_Yellow();
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
    kick_state  = STATE_IDLE;
    kick_result = KICK_RESULT_NONE;
    memset(&kick_dbg, 0, sizeof(kick_dbg));
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

    /* ---- STATE_IDLE: wait for key press ---- */
    if (kick_state == STATE_IDLE) return;

    /* ---- STATE_INIT: check horizontal posture (once only) ---- */
    if (kick_state == STATE_INIT)
    {
        if (init_checked)
        {
            kick_result = KICK_RESULT_NONE;
            return;   /* 已检测过，不再重复，等按键重新触发 */
        }
        init_checked = 1;

        float p = imu.pitch;
        float r = imu.roll;

        if (p < 0.0f) p = -p;
        if (r < 0.0f) r = -r;

        if (p <= POSTURE_HORIZONTAL_THRESHOLD_DEG &&
            r <= POSTURE_HORIZONTAL_THRESHOLD_DEG)
        {
            BSP_WS2812_Green();
            BSP_Beep_Short();
            baseline_roll  = imu.roll;
            baseline_pitch = imu.pitch;
            kick_set_state(STATE_POSTURE_OK);
        }
        else
        {
            BSP_WS2812_Red();
            BSP_Beep_Long();
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

    /* ---- STATE_DETECT: monitor kick motion (continuous) ---- */

    /* Cooldown: don't evaluate immediately after a previous evaluation */
    if (last_eval_tick != 0 &&
        (HAL_GetTick() - last_eval_tick) < KICK_EVAL_COOLDOWN_MS)
    {
        return;
    }

    /* ---- Kick cycle detection on vertical acceleration ---- */
    float acc_z_motion = imu.acc_bz;

    /* Start of kick cycle: acceleration deviates from gravity */
    if (!cycle_active)
    {
        float abs_acc = acc_z_motion;
        if (abs_acc < 0.0f) abs_acc = -abs_acc;

        if (abs_acc > KICK_START_THRESHOLD_MSS)
        {
            /* Kick cycle started — reset integration */
            cycle_active    = 1;
            cycle_start_tick = HAL_GetTick();
            acc_z_min  = acc_z_motion;  acc_z_max  = acc_z_motion;
            roll_min   = imu.roll;       roll_max   = imu.roll;
            vel_x = 0.0f;  vel_y = 0.0f;  vel_z = 0.0f;
            pos_x = 0.0f;  pos_y = 0.0f;  pos_z = 0.0f;
            pos_z_max = 0.0f;  pos_z_min = 0.0f;
            cycle_has_peak  = 0;
            cycle_has_trough = 0;
            last_sample_tick = imu.timestamp;
        }
        else
        {
            /* Not kicking — reset result for LED feedback */
            if (kick_result != KICK_RESULT_NONE &&
                (HAL_GetTick() - last_eval_tick) > 1000U)
            {
                kick_result = KICK_RESULT_NONE;
            }
        }
        return;
    }

    /* ---- Active cycle: integrate acceleration → velocity → position ---- */
    {
        float dt = 0.01f;
        if (last_sample_tick != 0 && imu.timestamp != last_sample_tick)
        {
            dt = (float)(imu.timestamp - last_sample_tick) / 1000.0f;
            if (dt > 0.5f) dt = 0.01f;
        }
        /* 梯形积分，位移统一 cm */
        vel_x += imu.acc_bx * dt;
        vel_y += imu.acc_by * dt;
        vel_z += acc_z_motion * dt;
        pos_x += vel_x * dt * 100.0f;
        pos_y += vel_y * dt * 100.0f;
        pos_z += vel_z * dt * 100.0f;
    }
    last_sample_tick = imu.timestamp;

    /* Update acc_z peaks */
    if (acc_z_motion > acc_z_max) acc_z_max = acc_z_motion;
    if (acc_z_motion < acc_z_min) acc_z_min = acc_z_motion;

    /* Track roll envelope */
    if (imu.roll > roll_max) roll_max = imu.roll;
    if (imu.roll < roll_min) roll_min = imu.roll;

    /* 位置峰谷 */
    if (pos_z > pos_z_max) pos_z_max = pos_z;
    if (pos_z < pos_z_min) pos_z_min = pos_z;

    /* 实时更新调试指标 */
    kick_dbg.force    = acc_z_max - acc_z_min;
    kick_dbg.disp_z   = (pos_z_max > pos_z_min) ? (pos_z_max - pos_z_min) : (pos_z_min - pos_z_max);
    kick_dbg.disp_x   = pos_x;
    kick_dbg.disp_y   = pos_y;
    kick_dbg.roll_chg = roll_max - roll_min;
    last_sample_tick = imu.timestamp;

    /* Detect peak/trough via velocity sign change (integration done above) */
    {
        static float vel_z_prev = 0.0f;

        if (vel_z_prev > 0.0f && vel_z <= 0.0f) cycle_has_peak   = 1;
        if (vel_z_prev < 0.0f && vel_z >= 0.0f) cycle_has_trough = 1;
        vel_z_prev = vel_z;
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
 * @brief  按键触发：进入姿态检测
 */
void KickDetect_StartInit(void)
{
    init_checked = 0;
    kick_set_state(STATE_INIT);
    kick_result = KICK_RESULT_NONE;
    kick_cycle_reset();
    BSP_Beep_Off();
}

/**
 * @brief  Reset to IDLE
 */
void KickDetect_Reset(void)
{
    kick_set_state(STATE_IDLE);
    kick_result = KICK_RESULT_NONE;
    kick_cycle_reset();
    BSP_Beep_Off();
}
