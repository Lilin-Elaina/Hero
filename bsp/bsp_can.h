#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"

void Filter_Init(void);
void FDCAN2_Start(void);
void FDCAN1_Start(void);

/* ---- 统一收发接口：上层（modules）不再直接碰 HAL_FDCAN ----
 * bus：1 = FDCAN1（pitch 6020 + 摩擦轮），2 = FDCAN2（底盘四轮 + yaw 6020）
 * 帧格式固定为“标准数据帧、8 字节”，与本工程现有所有报文一致。 */
HAL_StatusTypeDef BSP_CAN_Send(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t len);
void BSP_CAN_Stop(uint8_t bus);   /* yaw_kill 等安全兜底用：关掉总线输出 */

#endif
