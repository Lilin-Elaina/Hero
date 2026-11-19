#ifndef __BSP_TIME_H
#define __BSP_TIME_H

#include "main.h"

/* 时间基准（毫秒）。
 * 现在直接转发 HAL_GetTick/HAL_Delay：行为与原来逐位相同。
 * 以后要换 DWT 周期计数器时只改 bsp_time.c（注意 CYCCNT 是 32 位、
 * 480 MHz 下 8.95 s 回绕，必须做回绕累加，不能直接 /480000）。 */
uint32_t BSP_Now(void);
void     BSP_DelayMs(uint32_t ms);

#endif
