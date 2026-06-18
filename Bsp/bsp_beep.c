/**
 ******************************************************************************
 * @file    bsp_beep.c
 * @brief   Active buzzer — GPIO PD13, non-blocking
 *
 *          BSP_Beep_Short/Long 仅启动蜂鸣并记录开始时刻。
 *          BSP_Beep_Process 在主循环中检查是否到时，到时自动关停。
 *
 *          PD13 初始为 TIM4 CH2 AF_PP，这里重配为 GPIO 输出。
 *          必须在 MX_TIM4_Init() 之后调用。
 ******************************************************************************
 */

#include "bsp_beep.h"

static uint32_t beep_start_tick;
static uint32_t beep_duration_ms;
static uint8_t  beep_active;

void BSP_Beep_Init(void)
{
//    GPIO_InitTypeDef gpio = {0};
//    gpio.Pin   = GPIO_PIN_13;
//    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
//    gpio.Pull  = GPIO_NOPULL;
//    gpio.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(GPIOD, &gpio);
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

    beep_active = 0;
}

static void beep_start(uint32_t ms)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    beep_start_tick  = HAL_GetTick();
    beep_duration_ms = ms;
    beep_active      = 1;
}

void BSP_Beep_Short(void) { beep_start(BEEP_SHORT_MS); }
void BSP_Beep_Long(void)  { beep_start(BEEP_LONG_MS);  }

void BSP_Beep_Off(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    beep_active = 0;
}

void BSP_Beep_Process(void)
{
    if (!beep_active) return;
    if ((HAL_GetTick() - beep_start_tick) >= beep_duration_ms)
    {
        BSP_Beep_Off();
    }
}
