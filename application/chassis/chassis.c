/**
 * @file    chassis.c
 * @brief   底盘四轮：故障标志、积分状态、单轮速度 PI
 * @note    2026-11-19 分层整理：以下内容从 remote/rc_dbus.c 原样搬迁，逻辑未改。
 *          改动仅限：文件头注释、include、以及跨文件共享变量的存储类。
 */
#include "rc_dbus.h"
#include "rc_state.h"
#include "usart.h"
#include "DJ_Motor.h"
#include "mecanum.h"
#include "ins.h"
#include "fdcan.h"
#include "bsp_can.h"
#include <string.h>
#include <math.h>

/*=============底盘四轮速度闭环===========*/
 volatile uint8_t RC_WheelFault = 0; /*轮子故障标志位，1表示有轮子故障，0表示正常*/

/* 三层共享（robot.c 的失速兜底要清零积分）——只改存储类，语义不变 */
float s_wheel_i[MECANUM_WHEEL_NUM];

/* 目标轮速镜像（只给调试器看，控制里不读） */
float RC_WheelRef[4] = {0.0f};
/* 非 static：robot.c 的四轮循环也要用这两个小工具 */
float wheel_abs(float x )
{
    return (x < 0.0f) ? -x : x;
}

float wheel_clamp(float v, float lo, float hi)
{
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return v;
}

void RC_Wheel_Reset()
{
     uint8_t i;

     for(i = 0; i < MECANUM_WHEEL_NUM; i++)
     {
         s_wheel_i[i] = 0.0f;
     }
}

  /*单轮速度  PI  :前馈 + P +条件积分 + 输出限幅
   * 非 static：robot.c 的四轮循环要调用它（分层前是同文件，天然可见） */
float RC_Wheel_Speed_PI(uint8_t id, float ref , float rpm, float limit)
{
    float err  = ref - rpm;
    float a_ff = RC_WHEEL_FF_A_PER_RPM * ref;
    float a    = a_ff + RC_WHEEL_SPD_KP * err + s_wheel_i[id];

    if(wheel_abs(a) < limit || (a >= limit && err < 0.0f) || (a <= -limit && err > 0.0f))
    {
        s_wheel_i[id] += RC_WHEEL_SPD_KI * err * RC_WHEEL_SPD_DT_S;
        s_wheel_i[id] = wheel_clamp(s_wheel_i[id], -RC_WHEEL_I_MAX, RC_WHEEL_I_MAX);
    }

    a = a_ff + RC_WHEEL_SPD_KP * err + s_wheel_i[id];
    
    return wheel_clamp(a, -limit, limit);

}
