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

void BSP_IMU_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* 先把电平置高（未选中），再切成输出，避免切换瞬间误选通 */
    HAL_GPIO_WritePin(BSP_IMU_CS_PORT, BSP_IMU_CS_ACC | BSP_IMU_CS_GYRO, GPIO_PIN_SET);

    g.Pin   = BSP_IMU_CS_ACC | BSP_IMU_CS_GYRO;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BSP_IMU_CS_PORT, &g);

    /* H7 的 PC2_C/PC3_C 与 PC2/PC3 之间有模拟开关，不闭合则引脚不通 */
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);
}
/*imu片选，第一个参数选择设备，第二个为电平高低*/
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
