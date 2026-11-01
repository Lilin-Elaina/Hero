#ifndef __DJ_MOTOR_H
#define __DJ_MOTOR_H

#include "main.h"

HAL_StatusTypeDef DJ3508_Send_I_Can2(int16_t ID1_I,int16_t ID2_I,int16_t ID3_I,int16_t ID4_I);

HAL_StatusTypeDef DJ6020_Send_V_Can2(int16_t ID1_V, int16_t ID2_V);

/* CAN1 专用：摩擦轮 3508 ID2/ID3 + 云台 pitch 6020 ID2 */
HAL_StatusTypeDef DJ3508_Send_Can1(int16_t ID2_I, int16_t ID3_I);
HAL_StatusTypeDef DJ6020_Send_Can1(int16_t ID2_V);
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
extern volatile GM6020_State_t dj_wheel[4];  /* CAN2 上的 3508：底盘四轮 LF/LB/RB/RF（ID1~ID4） */
/* 诊断计数器：看 CAN 接收到底通不通 *//*排错结构体*/
extern volatile uint32_t can_rx_total;
extern volatile uint32_t can_rx_205;
extern volatile uint32_t can_rx_last_id;
extern volatile int16_t  can_rx_last_d0;

#endif
