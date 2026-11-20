#ifndef __GIMBAL_SPIN_H
#define __GIMBAL_SPIN_H

/* 小陀螺模式专用 yaw 控制器（application/gimbal/gimbal_spin.c）
 *
 * 与 gimbal.c 的 RC_Yaw_Control() 是**互斥的两条环**：
 *   跟随/手动 → RC_Yaw_Control()      （目标在车体系：保持关节角）
 *   小陀螺    → RC_Yaw_Spin_Control() （目标在世界系：保持枪口指向）
 * 同一拍只允许一条输出，靠 robot.c 里的 spin_mode 选择，别把两者相加。 */

#include <stdint.h>

/* 小陀螺：世界航向目标 + 底盘转速前馈，输出 6020 电压指令。
 * 内部维护 S_Spin_Ref / S_Spin_I / S_Spin_Ready。 */
float RC_Yaw_Spin_Control(uint8_t online, float stick);

/* 模式切换时调用：把两条环的内部状态都复位，
 * 避免上一条环的积分/目标带进下一条环（否则切模式瞬间会猛摆一下）。 */
void RC_Yaw_Handover(void);

#endif /* __GIMBAL_SPIN_H */
