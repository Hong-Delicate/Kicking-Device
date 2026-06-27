/**
 ******************************************************************************
 * @file    bsp_ws2812.h
 * @brief   Single WS2812 LED driver — TIM4 CH3 (PD14) PWM + DMA
 *
 * @note    CubeMX configures:
 *            - TIM4:   PSC=0, ARR=89 → 800kHz (MX_TIM4_Init)
 *            - DMA:    DMA1_Channel5, MEM→PERIPH, half-word
 *            - NVIC:   DMA1_Channel5_IRQn priority 0 (MX_DMA_Init)
 *            - GPIO:   PD14 = AF_OD (HAL_TIM_MspPostInit)
 *
 *          Single WS2812 LED, 24-bit GRB color. Hardware init by CubeMX.
 ******************************************************************************
 */

#ifndef __BSP_WS2812_H__
#define __BSP_WS2812_H__

#include "main.h"
#include "tim.h"

/* -------------------------------------------------------------------------- */
/*  WS2812 timing                                                             */
/* -------------------------------------------------------------------------- */

#define WS2812_TIM_PERIOD           89U     /* ARR → 72MHz/(89+1) = 800kHz */
#define WS2812_TIM_PRESCALER        0U
#define WS2812_BIT0_CCR             30U     /* ~0.42us high (0-code) */
#define WS2812_BIT1_CCR             60U     /* ~0.83us high (1-code) */
#define WS2812_RESET_CYCLES         300U     /* >50us reset → 62.5us */

/* DMA buffer: 24 bits for 1 LED + reset gap */
#define WS2812_BUF_SIZE             (24U + WS2812_RESET_CYCLES)

/* -------------------------------------------------------------------------- */
/*  CubeMX DMA handle (defined in tim.c)                                      */
/* -------------------------------------------------------------------------- */

extern DMA_HandleTypeDef hdma_tim4_ch3;

/* -------------------------------------------------------------------------- */
/*  Color type (GRB byte order: G→R→B)                                       */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint8_t g;
    uint8_t r;
    uint8_t b;
} WS2812_Color_t;

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

void     BSP_WS2812_Init(void);
void     BSP_WS2812_SetColor(uint8_t red, uint8_t green, uint8_t blue);
void     BSP_WS2812_Update(void);
uint8_t  BSP_WS2812_IsBusy(void);
void     BSP_WS2812_Off(void);

void     BSP_WS2812_RestoreTimConfig(void);   /* TIM4 restore after beep */

/* Color presets (set + update in one call) */
void     BSP_WS2812_Red(void);
void     BSP_WS2812_Green(void);
void     BSP_WS2812_Yellow(void);

#endif /* __BSP_WS2812_H__ */
