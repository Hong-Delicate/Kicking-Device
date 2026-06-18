/**
 ******************************************************************************
 * @file    bsp_key.h
 * @brief   Key input via GPIO PB14 with software debounce
 ******************************************************************************
 */

#ifndef __BSP_KEY_H__
#define __BSP_KEY_H__

#include "main.h"

/* Debounce time in milliseconds */
#define KEY_DEBOUNCE_MS     30U

/* Key state enumeration */
typedef enum {
    KEY_STATE_IDLE = 0,
    KEY_STATE_PRESSED,
    KEY_STATE_RELEASED
} KeyState_t;

void BSP_Key_Init(void);
void BSP_Key_Update(void);          /* Call periodically (~10ms) to update debounce FSM */
KeyState_t BSP_Key_Read(void);      /* Read and clear current key event */
uint8_t BSP_Key_IsPressed(void);    /* Raw key state (1 = pressed, ignores debounce) */

#endif /* __BSP_KEY_H__ */
