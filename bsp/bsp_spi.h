#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "main.h"

/* 本工程只有一条 SPI 总线（SPI2）挂 BMI088：
 *   加速度计 CS = PC0，陀螺 CS = PC3，两片各自选通。 */
#define BSP_IMU_DEV_ACC    0u
#define BSP_IMU_DEV_GYRO   1u

/* 片选：low!=0 拉低（选中），low==0 拉高（释放）。
 * 片选脚与 H7 的 PC2_C/PC3_C 模拟开关由 CubeMX（gpio.c）配置，不再需要 Init。 */
void BSP_IMU_Cs(uint8_t dev, uint8_t low);

/* 全双工传输 n 字节，返回 1 成功 / 0 失败（超时 50 ms）。
 * 调用者自己负责片选，因为“片选—传输—释放”要作为一个整体。 */
uint8_t BSP_IMU_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t n);

/* 只发不收（写寄存器用），返回 1 成功 / 0 失败（超时 20 ms） */
uint8_t BSP_IMU_Send(const uint8_t *tx, uint16_t n);

#endif
