#ifndef __GIMBAL_H
#define __GIMBAL_H

/* 云台层对外接口（application/gimbal/gimbal.c 的实现）
 *
 * 包含 pitch 位置环与 yaw 车体系角度环；小陀螺那一套在 gimbal_spin.h。
 * 参数宏仍然在 application/rc_dbus.h。 */

#include <stdint.h>

/* pitch 位置环：摇杆直接给角度增量，带重力前馈。
 * 返回 6020 的电压指令（已含方向宏与限幅）。 */
float RC_Pitch_Control(uint8_t online, uint8_t armed, float stick);

/* 枪口相对车体的偏转角（度，-180~180）。
 * 同时被跟随环（算 α）和枪口系平移（旋转矩阵）使用；
 * 编码器反馈超时时返回 0，避免用冻结的角度继续追。 */
float RC_GetOffsetAngle(void);

/* yaw 车体系角度环：摇杆积分出目标关节角，P/D 位置环 + 速度环。
 * 返回 6020 的电压指令（已含斜坡前的限幅）。 */
float RC_Yaw_Control(uint8_t online, float stick);

float pitch_clamp(float v, float lo, float hi);
#endif /* __GIMBAL_H */
