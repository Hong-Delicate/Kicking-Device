/**
 ******************************************************************************
 * @file    bsp_motor.c
 * @brief   Vibration motor control via GPIO PD12
 *
 * @note    De-initializes PD12 from TIM4 CH1 AF_PP mode (set by CubeMX in
 *          HAL_TIM_MspPostInit) and reconfigure as GPIO output push-pull.
 *          Must be called AFTER MX_TIM4_Init().
 ******************************************************************************
 */

#include "bsp_motor.h"

/**
 * @brief  Initialize motor GPIO
 * @note   Overrides CubeMX AF_PP configuration for TIM4 CH1 on PD12
 */
void BSP_Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Re-init PD12 as GPIO output (override CubeMX AF_PP setting for TIM4 CH1) */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Ensure motor starts off */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
}

/**
 * @brief  Turn motor on (full vibration)
 */
void BSP_Motor_On(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
}

/**
 * @brief  Turn motor off
 */
void BSP_Motor_Off(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
}

/**
 * @brief  Trigger a vibration pulse (200ms, blocking)
 */
void BSP_Motor_Vibrate(void)
{
    BSP_Motor_On();
    HAL_Delay(MOTOR_VIBRATE_MS);
    BSP_Motor_Off();
}
