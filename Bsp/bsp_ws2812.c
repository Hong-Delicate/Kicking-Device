/**
 ******************************************************************************
 * @file    bsp_ws2812.c
 * @brief   Single WS2812 LED driver — TIM4 CH3 PWM + DMA
 *
 * @note    CubeMX handles all hardware init (TIM4, DMA, NVIC, GPIO).
 *          This driver manages one LED's color state and GRB bitstream.
 ******************************************************************************
 */

#include "bsp_ws2812.h"

/* -------------------------------------------------------------------------- */
/*  Internal state                                                            */
/* -------------------------------------------------------------------------- */

static WS2812_Color_t   ws2812_led;
static uint16_t         ws2812_dma_buf[WS2812_BUF_SIZE];
volatile uint8_t ws2812_busy;           /* 调试可见 */
volatile uint16_t ws2812_dma_cnt;       /* DMA 完成计数 */

/* -------------------------------------------------------------------------- */
/*  GRB bitstream encoding (single LED, 24 bits)                              */
/* -------------------------------------------------------------------------- */

static void ws2812_encode_buffer(void)
{
    uint16_t idx = 0;

    /* GRB byte order, MSB first per byte */
    uint8_t bytes[3] = { ws2812_led.g, ws2812_led.r, ws2812_led.b };

    for (uint8_t b = 0; b < 3; b++)
    {
        for (int8_t bit = 7; bit >= 0; bit--)
        {
            ws2812_dma_buf[idx++] = (bytes[b] & (1U << bit))
                                    ? WS2812_BIT1_CCR : WS2812_BIT0_CCR;
        }
    }

    /* Reset gap: CCR=0 for >50us */
    for (uint16_t i = 0; i < WS2812_RESET_CYCLES; i++)
    {
        ws2812_dma_buf[idx++] = 0;
    }
}

/* -------------------------------------------------------------------------- */
/*  TIM4 restore — called by bsp_beep after beep                              */
/* -------------------------------------------------------------------------- */

//void BSP_WS2812_RestoreTimConfig(void)
//{
//    TIM_OC_InitTypeDef sConfigOC = {0};

//    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);

//    __HAL_TIM_SET_PRESCALER(&htim4, WS2812_TIM_PRESCALER);
//    __HAL_TIM_SET_AUTORELOAD(&htim4, WS2812_TIM_PERIOD);
//    __HAL_TIM_SET_COUNTER(&htim4, 0);

//    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse      = 0;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
//}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

void BSP_WS2812_Init(void)
{
//    /* CubeMX 设了 PD14=AF_OD，WS2812 需要推挽 → 改为 AF_PP */
//    GPIO_InitTypeDef gpio = {0};
//    gpio.Pin   = GPIO_PIN_14;
//    gpio.Mode  = GPIO_MODE_AF_PP;
//    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOD, &gpio);

    ws2812_led.g = 0;
    ws2812_led.r = 0;
    ws2812_led.b = 0;
    ws2812_busy = 0;
    ws2812_dma_cnt = 0;
    BSP_WS2812_Update();
}

void BSP_WS2812_SetColor(uint8_t red, uint8_t green, uint8_t blue)
{
    ws2812_led.g = green;
    ws2812_led.r = red;
    ws2812_led.b = blue;
}

void BSP_WS2812_Update(void)
{
    while (ws2812_busy) { }     /* Wait for previous transfer (~30us) */

    ws2812_busy = 1;
    ws2812_encode_buffer();
    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_3,
                          (uint32_t *)ws2812_dma_buf, WS2812_BUF_SIZE);
}

uint8_t BSP_WS2812_IsBusy(void)
{
    return ws2812_busy;
}

void BSP_WS2812_Off(void)
{
    BSP_WS2812_SetColor(0, 0, 0);
    BSP_WS2812_Update();
}

/* ---- Presets ---- */

void BSP_WS2812_Red(void)
{
    BSP_WS2812_SetColor(255, 0, 0);     /* GRB: R=255 */
    BSP_WS2812_Update();
}

void BSP_WS2812_Green(void)
{
    BSP_WS2812_SetColor(0, 255, 0);     /* GRB: G=255 */
    BSP_WS2812_Update();
}

void BSP_WS2812_Yellow(void)
{
    BSP_WS2812_SetColor(255, 255, 0);   /* GRB: R=255, G=255 */
    BSP_WS2812_Update();
}

/* -------------------------------------------------------------------------- */
/*  HAL callback: DMA transfer complete                                       */
/* -------------------------------------------------------------------------- */

//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM4)
//    {
//        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0);
//        ws2812_busy = 0;
//        ws2812_dma_cnt++;
//    }
//}
