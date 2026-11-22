/**
 * @file    rc_state.h
 * @brief   跨文件共享的 yaw 环状态（分层后暴露给 gimbal_spin.c / robot.c）
 * @note    这些变量原本是 remote/rc_dbus.c 里的文件级 static。按“只分层不改逻辑”
 *          的约定，唯一允许的改动就是把它们的存储类从 static 改为全局，并在
 *          定义处保留原类型与初值。见 application/gimbal/gimbal.c 与
 *          application/gimbal/gimbal_spin.c 的定义处注释。
 */
#ifndef __RC_STATE_H
#define __RC_STATE_H



#include <stdint.h>

/* 定义在 application/gimbal/gimbal.c —— yaw 车体系角度环 */
extern float   S_Yaw_Ref;      /* 目标角（counts） */
extern float   S_Yaw_I;        /* 速度环积分（电压单位） */
extern uint8_t S_Yaw_Ready;    /* 目标角初始化完成标志 */

/* 定义在 application/gimbal/gimbal_spin.c —— 交接后要求把输出斜坡清零 */
extern uint8_t S_Yaw_Slew_Kick;

/* 定义在 application/chassis/chassis.c —— 四轮速度环积分（失速兜底要清零） */
extern float S_Wheel_I[];


#endif /* __RC_STATE_H */
