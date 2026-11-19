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

/* ---- 接收：bsp 只负责“从 FIFO 取帧 + 按总线派发”，报文含义留给 modules ----
 * 这样 bsp 不认识任何电机/遥控器，modules 也不用再写 HAL 回调。 */
typedef void (*BSP_CAN_RxHandler)(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t len);
void BSP_CAN_RegisterRx(uint8_t bus, BSP_CAN_RxHandler handler);

/* 总线级统计（随接收路径一起下沉到 bsp；名字不变，Ozone 里照样能看） */
extern volatile uint32_t can_rx_total;         /* 两路合计帧数 */
extern volatile uint32_t can_rx2_total;        /* 仅 CAN2 帧数（活性看门狗用） */
extern volatile uint32_t can_rx_last_id;       /* 最近一帧 ID */
extern volatile int16_t  can_rx_last_d0;       /* 最近一帧 data[0] */
extern volatile uint32_t can_rx_id_cnt[8];     /* 0x201..0x208 分别计数 */

#endif
