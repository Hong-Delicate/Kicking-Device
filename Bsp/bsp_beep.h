/**
 ******************************************************************************
 * @file    bsp_beep.h
 * @brief   Active buzzer — GPIO PD13, non-blocking (SysTick driven)
 ******************************************************************************
 */

#ifndef __BSP_BEEP_H__
#define __BSP_BEEP_H__

#include "main.h"

#define BEEP_SHORT_MS  200U
#define BEEP_LONG_MS   500U

void BSP_Beep_Init(void);
void BSP_Beep_Short(void);
void BSP_Beep_Long(void);
void BSP_Beep_Off(void);
void BSP_Beep_Process(void);    /* 主循环调用，非阻塞 */;

#endif
