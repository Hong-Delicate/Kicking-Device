/**
 ******************************************************************************
 * @file    kick_detect.h
 * @brief   Freestyle kick detection algorithm
 *
 * @note    State machine that evaluates IMU data to detect swimming kick
 *          quality. Provides visual (WS2812 LED), audible (buzzer), and
 *          haptic (motor) feedback based on kick performance.
 ******************************************************************************
 */

#ifndef __KICK_DETECT_H__
#define __KICK_DETECT_H__

#include "main.h"

/* -------------------------------------------------------------------------- */
/*  Tunable algorithm parameters                                              */
/* -------------------------------------------------------------------------- */

/* Posture: pitch/roll must be within ±threshold to be "horizontal" (degrees) */
#define POSTURE_HORIZONTAL_THRESHOLD_DEG      15.0f

/* Kick amplitude: peak-to-peak vertical displacement (cm) */
#define KICK_AMPLITUDE_THRESHOLD_CM           30.0f

/* Flip detection: roll angle change during one kick cycle (degrees) */
#define FLIP_ROLL_THRESHOLD_DEG               30.0f

/* Horizontal displacement threshold (m) — estimated lateral drift */
#define HORIZONTAL_DISPLACEMENT_THRESHOLD_M   0.5f

/* Kick force proxy: peak vertical acceleration (m/s²) */
#define KICK_FORCE_WEAK_THRESHOLD             10.0f
#define KICK_FORCE_STRONG_THRESHOLD           25.0f

/* Detection window: how long to accumulate data before evaluating (ms) */
#define SAMPLE_WINDOW_MS                      2000U

/* Timeout: return to INIT if no kick detected for this long (ms) */
#define KICK_DETECT_TIMEOUT_MS                5000U

/* Minimum samples between kick evaluations (debounce, ms) */
#define KICK_EVAL_COOLDOWN_MS                 500U

/* -------------------------------------------------------------------------- */
/*  State machine                                                             */
/* -------------------------------------------------------------------------- */

typedef enum {
    STATE_INIT = 0,         /* Checking initial horizontal posture */
    STATE_POSTURE_OK,       /* Posture confirmed, monitoring starts */
    STATE_DETECT            /* Active kick detection */
} KickState_t;

typedef enum {
    KICK_RESULT_NONE = 0,       /* No kick detected / evaluating */
    KICK_RESULT_GOOD,           /* Kick >= 30cm, standard form */
    KICK_RESULT_BAD_AMPLITUDE,  /* Kick < 30cm */
    KICK_RESULT_BAD_FORM        /* Kick >= 30cm but flip/excessive lateral movement */
} KickResult_t;

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

void         KickDetect_Init(void);
void         KickDetect_Process(void);
KickState_t  KickDetect_GetState(void);
KickResult_t KickDetect_GetResult(void);
void         KickDetect_Reset(void);

#endif /* __KICK_DETECT_H__ */
