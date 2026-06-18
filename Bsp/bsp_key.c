/**
 ******************************************************************************
 * @file    bsp_key.c
 * @brief   Key input via GPIO PB14 with software debounce
 *
 * @note    PB14 is initially configured as GPIO input no-pull by CubeMX.
 *          We reconfigure with internal pull-up for stable idle state.
 *          Assumes key is active-low (pressed = GND, released = VCC via pull-up).
 ******************************************************************************
 */

#include "bsp_key.h"

/* Debounce state machine variables */
static uint32_t    key_debounce_tick = 0;
static uint8_t     key_last_raw = 1;         /* Last raw read (1 = idle with pull-up) */
static uint8_t     key_stable_state = 1;     /* Debounced stable state */
 KeyState_t  key_event = KEY_STATE_IDLE;

/**
 * @brief  Initialize key GPIO with internal pull-up
 */
void BSP_Key_Init(void)
{
//    GPIO_InitTypeDef GPIO_InitStruct = {0};

//    /* Re-init PB14 as input with pull-up (CubeMX set NOPULL) */
//    GPIO_InitStruct.Pin = GPIO_PIN_14;
//    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Initialize debounce state */
    key_debounce_tick = HAL_GetTick();
    key_last_raw = 1;
    key_stable_state = 1;
    key_event = KEY_STATE_IDLE;
}

/**
 * @brief  Update debounce state machine (call every ~10ms from main loop)
 *
 * Implements a simple integrate-and-dump debounce:
 *  - Sample raw pin level
 *  - If it differs from last_raw, reset debounce timer
 *  - If it stays stable for KEY_DEBOUNCE_MS, update stable_state
 *  - Detect transitions on stable_state and set key_event
 */
void BSP_Key_Update(void)
{
    uint8_t raw = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) ? 0 : 1;

    /* Raw level changed → reset debounce timer */
    if (raw != key_last_raw)
    {
        key_debounce_tick = HAL_GetTick();
        key_last_raw = raw;
        return;
    }

    /* Raw level stable for debounce period → update stable state */
    if ((HAL_GetTick() - key_debounce_tick) >= KEY_DEBOUNCE_MS)
    {
        if (raw != key_stable_state)
        {
            key_stable_state = raw;

            /* Edge detected on stable signal */
            if (key_stable_state == 0)
            {
                /* 1 → 0: key pressed */
                key_event = KEY_STATE_PRESSED;
            }
            else
            {
                /* 0 → 1: key released */
                key_event = KEY_STATE_RELEASED;
            }
        }
    }
}

/**
 * @brief  Read and clear the current key event
 * @return KeyState_t: KEY_STATE_PRESSED, KEY_STATE_RELEASED, or KEY_STATE_IDLE
 */
KeyState_t BSP_Key_Read(void)
{
    KeyState_t evt = key_event;
    key_event = KEY_STATE_IDLE;
    return evt;
}

/**
 * @brief  Get raw key state (un-debounced)
 * @return 1 if currently pressed, 0 if released
 */
uint8_t BSP_Key_IsPressed(void)
{
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) ? 1 : 0;
}
