/**
 ******************************************************************************
 * @file    bsp_imu.c
 * @brief   IMU 驱动 — HiPNUC CH010 (hipnuc_dec 库)
 *
 *          HAL_UART_Receive_IT 逐字节接收 → hipnuc_input 解码 →
 *          完整 0x91 包 → 填充 IMU_Data_t。
 ******************************************************************************
 */

#include "bsp_imu.h"
#include "usart.h"
#include "hipnuc_dec.h"

#define GRAVITY  9.8f

static hipnuc_raw_t     imu_raw;
IMU_Data_t              imu_data;
volatile uint8_t        imu_data_ready;
static uint8_t          imu_rx_byte;

/* 去重力：用 roll/pitch 估算机体坐标系下的重力分量并减去 */
float gx_est, gy_est, gz_est;           /* 重力估算 (m/s²), 调试可见 */
uint8_t g_calibrated;                   /* 收敛标志, 调试可见 */

/* 低通滤波估算重力向量（时间常数 ~0.5s @100Hz） */
#define G_ALPHA  0.98f

static void remove_gravity(IMU_Data_t *d)
{
    if (!g_calibrated) {
        /* 前几帧直接取原始值作为初始估计 */
        gx_est = d->acc_x;
        gy_est = d->acc_y;
        gz_est = d->acc_z;
        g_calibrated = 1;
    } else {
        gx_est = G_ALPHA * gx_est + (1.0f - G_ALPHA) * d->acc_x;
        gy_est = G_ALPHA * gy_est + (1.0f - G_ALPHA) * d->acc_y;
        gz_est = G_ALPHA * gz_est + (1.0f - G_ALPHA) * d->acc_z;
    }

    d->acc_bx = d->acc_x - gx_est;
    d->acc_by = d->acc_y - gy_est;
    d->acc_bz = d->acc_z - gz_est;
}

/* ---- HAL 回调 ---- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;

    if (hipnuc_input(&imu_raw, imu_rx_byte) > 0)
    {
        if (imu_raw.hi91.tag == 0x91)
        {
            imu_data.roll      = imu_raw.hi91.roll;
            imu_data.pitch     = imu_raw.hi91.pitch;
            imu_data.yaw       = imu_raw.hi91.yaw;
            imu_data.acc_x     = imu_raw.hi91.acc[0] * 9.8f;
            imu_data.acc_y     = imu_raw.hi91.acc[1] * 9.8f;
            imu_data.acc_z     = imu_raw.hi91.acc[2] * 9.8f;
            imu_data.gyr_x     = imu_raw.hi91.gyr[0];
            imu_data.gyr_y     = imu_raw.hi91.gyr[1];
            imu_data.gyr_z     = imu_raw.hi91.gyr[2];
            imu_data.timestamp = imu_raw.hi91.system_time;
            remove_gravity(&imu_data);
            imu_data_ready     = 1;
        }
    }
    HAL_UART_Receive_IT(&huart2, &imu_rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        HAL_UART_Receive_IT(&huart2, &imu_rx_byte, 1);
    }
}

/* ---- Public API ---- */

void BSP_IMU_Init(void)
{
    imu_data_ready = 0;
    memset(&imu_raw, 0, sizeof(imu_raw));
    HAL_UART_Receive_IT(&huart2, &imu_rx_byte, 1);
}

uint8_t BSP_IMU_DataReady(void)
{
    return imu_data_ready;
}

void BSP_IMU_ClearDataReady(void)
{
    imu_data_ready = 0;
}

void BSP_IMU_GetData(IMU_Data_t *data)
{
    if (data) *data = imu_data;
}
