#ifndef __MECANUM_H
#define __MECANUM_H

#include "main.h"

//预编译，设定轮子宏定义，进行控制
#define MECANUM_WHEEL_NUM   4

#if (MECANUM_WHEEL_NUM == 2)
  #define MEC_WHEEL_L       0     /* 左轮索引 */
  #define MEC_WHEEL_R       1     /* 右轮索引 */
#else
  #define MEC_WHEEL_LF      0
  #define MEC_WHEEL_LB      1
  #define MEC_WHEEL_RB      2
  #define MEC_WHEEL_RF      3
#endif

/* ==================================================================
 * 进行比例设定，控制各类运动参数与摇杆值的关系，控制摇杆值所对应的电流值
 * ================================================================== */
#define MEC_MAX_LINEAR      1.0f    /*平移摇杆值即对应所给电流值*/
#define MEC_MAX_SPIN        1.0f    /*旋转摇杆值即对应所给电流值*/
#define MEC_SPIN_RATIO      0.60f   /* 旋转的电流权重*/
#define MEC_STRAFE_RATIO    0.85f   /*横移的电流权重*/

#define MEC_ACCEL_NORM      1.5f    /*每1ms加速的归一化值*/
#define MEC_STOP_NORM       4.0f    /*每1ms减速的归一化值*/
#define MEC_DEADBAND_NORM   0.02f   /*归一化后死区的比例*/

typedef struct
{
    float fwd;      /* 前进方向符号 */
    float strafe;   /* 横移方向符号 */
    float spin;     /* 自转方向符号 */
} Mecanum_Dir_t;

//结构体数组
extern const Mecanum_Dir_t MECANUM_DIR[MECANUM_WHEEL_NUM];

/* ==================================================================
 *  混控任务
 *  输入：fwd/turn 为摇杆归一化值 [-1,1]（已含死区）
 *        strafe   为横移摇杆归一化值，2 轮时填 0
 *        spin     为旋转摇杆归一化值，2 轮时与 turn 共用（填同一个值即可）
 *  输出：out[0..N-1] = 各轮归一化目标 [-1,1]
 *  内部含斜坡限速；失能时（enabled=0）按 MEC_STOP_NORM 快速回零后保持
 * ================================================================== */
void Mecanum_Mix(float fwd, float strafe, float spin, uint8_t enabled,
                 float *out);

/* 强制清掉斜坡内部状态（上电/模式切换时用，避免保留旧指令） */
void Mecanum_Reset(void);

#endif
