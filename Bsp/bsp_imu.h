/**
 ******************************************************************************
 * @file    bsp_imu.h
 * @brief   IMU 驱动 — HiPNUC CH010 帧解析
 ******************************************************************************
 */

#ifndef __BSP_IMU_H__
#define __BSP_IMU_H__

#include "main.h"

typedef struct {
    float    roll, pitch, yaw;
    float    acc_x, acc_y, acc_z;       /* 原始加速度 (m/s², 含重力) */
    float    acc_bx, acc_by, acc_bz;    /* 机体加速度 (m/s², 去重力) */
    float    gyr_x, gyr_y, gyr_z;
    uint32_t timestamp;
} IMU_Data_t;

void     BSP_IMU_Init(void);
uint8_t  BSP_IMU_DataReady(void);
void     BSP_IMU_ClearDataReady(void);
void     BSP_IMU_GetData(IMU_Data_t *data);

#endif
