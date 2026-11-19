#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"

void FDCAN2_Start(void);
void FDCAN1_Start(void);

/* ---- 统一收发接口：上层（modules）不再直接碰 HAL_FDCAN ----
 * bus：1 = FDCAN1（pitch 6020 + 摩擦轮），2 = FDCAN2（底盘四轮 + yaw 6020）
 * 本工程所有报文都是“标准数据帧、8 字节”，所以数据长度写死，不再传 len。 */
HAL_StatusTypeDef BSP_CAN_Send(uint8_t bus, uint32_t id, const uint8_t *data);

/* 关掉整条总线（不再发任何帧）。
 * 用途只有一个：故障兜底——yaw 的 P 环被判定失效时，与其发 0 电流，
 * 不如让总线停掉、让电调自己按超时收油，避免“发 0 但环还在算”导致抖。
 * 注意：停掉后恢复要靠 FDCANx_Start()，不是自动的。 */
void BSP_CAN_Stop(uint8_t bus);

/* ---- 接收：bsp 只负责“从 FIFO 取帧 + 按总线派发”，报文含义留给 modules ----
 * 这样 bsp 不认识任何电机/遥控器，modules 也不用再写 HAL 回调。 */

/* 接收回调类型（自定义，不是 HAL 或 CMSIS 的东西）：
 *   void 你的函数名(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t len)
 * 参数由 bsp 填：bus 是收到的总线号，id 是标准帧 ID，data 指向 8 字节负载，
 * len 目前恒为 8（保留是为了以后接非 8 字节报文）。 */
typedef void (*BSP_CAN_RxHandler)(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t len);

/* 给某条总线挂一个接收回调（由 DJ_Motor_Init() 调用）。
 * 同一个 bus 后注册的会覆盖先注册的；bsp 只会调用你挂的那个函数，
 * 所以“哪个 ID 是什么意思”完全由 modules 决定，bsp 不需要知道。 */
void BSP_CAN_RegisterRx(uint8_t bus, BSP_CAN_RxHandler handler);

/* 总线级统计（随接收路径一起下沉到 bsp；名字不变，Ozone 里照样能看） */
extern volatile uint32_t can_rx_total;         /* 两路合计帧数 */
extern volatile uint32_t can_rx2_total;        /* 仅 CAN2 帧数（活性看门狗用） */
extern volatile uint32_t can_rx_last_id;       /* 最近一帧 ID */
extern volatile int16_t  can_rx_last_d0;       /* 最近一帧 data[0] */
extern volatile uint32_t can_rx_id_cnt[8];     /* 0x201..0x208 分别计数 */

#endif
