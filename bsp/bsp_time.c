/**
 * @file    bsp_time.c
 * @brief   毫秒时间基准（上层不再直接调 HAL_GetTick / HAL_Delay）
 */
#include "main.h"
#include "bsp_time.h"

uint32_t BSP_Now(void)
{
    return HAL_GetTick();
}

void BSP_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}
