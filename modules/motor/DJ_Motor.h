#ifndef __DJ_MOTOR_H
#define __DJ_MOTOR_H

#include "main.h"

HAL_StatusTypeDef DJ3508_Send_I_Can2(int16_t ID1_I,int16_t ID2_I,int16_t ID3_I,int16_t ID4_I);

HAL_StatusTypeDef DJ6020_Send_V_Can2(int16_t ID1_V, int16_t ID2_V);

/* CAN1 专用：摩擦轮 3508 ID2/ID3 + 云台 pitch 6020 ID2 */
HAL_StatusTypeDef DJ3508_Send_Can1(int16_t ID2_I, int16_t ID3_I);
HAL_StatusTypeDef DJ6020_Send_Can1(int16_t ID2_V);

/* 在 CAN 启动之前调用一次：把接收解析函数注册给 bsp 层 */
void DJ_Motor_Init(void);
/* GM6020 ID1 的实时状态（由 CAN 接收回调更新） *//* 电机状态反馈*/
typedef struct
{
    uint16_t angle;      /* 机械角度 0~8191 */
    int16_t  rpm;        /* 转速 */
    int16_t  current;    /* 转矩电流 */
    uint8_t  temp;       /* 温度 */
    uint32_t last_tick;  /* 最后一帧的时刻 */
} GM6020_State_t;

extern volatile GM6020_State_t gim6020;
extern volatile GM6020_State_t gim_pitch;
extern volatile GM6020_State_t dj_motor[4];  /* CAN1 上的 3508：ID1~ID4（本工程用 ID2/ID3 摩擦轮） */
extern volatile GM6020_State_t dj_wheel[4];  /* CAN2 上的 3508：底盘四轮 LF/RF/RB/LB（ID1~ID4） */
/* 诊断计数器：只看 0x205（yaw）这一路，其余总线级计数已随 bsp 层简化删掉。
 * 活性判断改用 gim6020.last_tick 的年龄，不依赖帧计数。 */
extern volatile uint32_t can_rx_205;           /* 0x205 的帧数 */
extern volatile uint32_t can_rx_205_bad;      /* 0x205 中 |rpm|>600（6020 物理不可能）被整帧丢弃的帧数 */
extern volatile int16_t  can_rx_205_bad_rpm;   /* 最近一帧被丢弃时帧内的 rpm 原始值 */

#endif
