/**
 ******************************************************************************
 * @file    bsp_motor.h
 * @brief   Vibration motor control via GPIO PD12
 ******************************************************************************
 */

#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

#include "main.h"

/* Motor vibration duration (milliseconds) */
#define MOTOR_VIBRATE_MS    200U

void BSP_Motor_Init(void);
void BSP_Motor_On(void);
void BSP_Motor_Off(void);
void BSP_Motor_Vibrate(void);       /* On → delay(MOTOR_VIBRATE_MS) → Off, blocking */

#endif /* __BSP_MOTOR_H__ */
