#ifndef __CHASSIS_H
#define __CHASSIS_H

/* 底盘层对外接口（application/chassis/chassis.c 的实现）
 *
 * 这里只放"别的文件要调用底盘做什么"，参数宏仍然在 application/rc_dbus.h。
 * 依赖方向：本头文件不 include 任何东西（只用 stdint 的定长类型），
 * 这样 modules/bsp 永远不会把 application 拖进来。 */

#include <stdint.h>

/* 绝对值 / 限幅：底盘环内部用得很碎，提到头里给 robot.c 复用 */
float wheel_abs(float x);
float wheel_clamp(float v, float lo, float hi);

/* 清四个轮子的速度环积分（松杆、失联、切模式时必须调用，
 * 否则积分残留会让轮子"自己转"） */
void RC_Wheel_Reset(void);

/* 单轮速度环：ref 为目标转速(rpm)，rpm 为实测转速，
 * limit 为该轮允许的最大电流(A)。返回电流指令(A)。 */
float RC_Wheel_Speed_PI(uint8_t id, float ref, float rpm, float limit);

#endif /* __CHASSIS_H */
