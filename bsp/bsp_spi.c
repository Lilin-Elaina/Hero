/**
 * @file    bsp_spi.c
 * @brief   SPI2 + BMI088 片选（片上外设细节）
 * @note    2026-11-19 P2d：从 modules/imu/ins.c 原样搬迁，时序未改。
 *          模块层只调 BSP_IMU_Cs / BSP_IMU_Transfer / BSP_IMU_Send，
 *          不再直接碰 HAL_GPIO / HAL_SPI。
 */
#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "bsp_spi.h"

/* 片选脚：PC0 = 加速度计，PC3 = 陀螺 */
#define BSP_IMU_CS_PORT   GPIOC
#define BSP_IMU_CS_ACC    GPIO_PIN_0
#define BSP_IMU_CS_GYRO   GPIO_PIN_3

/* 片选脚（PC0/PC3）与 PC2_C 模拟开关都由 CubeMX 的 MX_GPIO_Init()（gpio.c）配置，
 * 执行顺序在 main.c 里早于 INS_Init()，本文件不再重复配置，只负责片选与收发。 */

/*imu片选，第一个参数选择设备，第二个为电平高低*/
//第二个参数为1是拉低片选，0是拉高片选
void BSP_IMU_Cs(uint8_t dev, uint8_t low)
{
    const uint16_t pin = (dev == BSP_IMU_DEV_ACC) ? BSP_IMU_CS_ACC : BSP_IMU_CS_GYRO;

    HAL_GPIO_WritePin(BSP_IMU_CS_PORT, pin, low ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
/*收发，用于读寄存器*/
uint8_t BSP_IMU_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    return (HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, n, 50) == HAL_OK) ? 1u : 0u;
}
/*写寄存器*/
uint8_t BSP_IMU_Send(const uint8_t *tx, uint16_t n)
{
    return (HAL_SPI_Transmit(&hspi2, (uint8_t *)tx, n, 20) == HAL_OK) ? 1u : 0u;
}
