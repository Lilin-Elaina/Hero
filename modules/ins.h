/**
  ******************************************************************************
  * @file    ins.h
  * @brief   BMI088 姿态采集（SPI2，双片选）
  *
  *  数据来源：达妙 DM-MC02 板载 BMI088
  *    SPI2 : SCK=PB13  MISO=PC2_C  MOSI=PC1
  *    CS   : 加速度计=PC0  陀螺=PC3   （低有效）
  *
  *  调用方式（1 kHz）：
  *    main() 初始化后调 INS_Init();
  *    主循环 1ms 分支里、RC_Control_Task() 之前调 INS_Task();
  *
  *  注意：BMI088 加速度计上电默认是 I2C 模式，
  *        靠 CS 上的一个上升沿切到 SPI —— INS_Init() 里已经处理。
  ******************************************************************************
  */
#ifndef __INS_H__
#define __INS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---------------- 调试观察量（Ozone 直接写表达式） ---------------- */
extern volatile uint8_t  INS_AccId;      /* 期望 0x1E */
extern volatile uint8_t  INS_GyroId;     /* 期望 0x0F */

extern volatile uint8_t  INS_Ready;      /* 1 = 初始化完成，Task 才会读数据 */
extern volatile uint8_t  INS_CalDone;    /* 1 = 零偏标定完成 */
extern volatile uint16_t INS_CalCnt;     /* 标定已采样拍数 */
extern volatile uint32_t INS_Sample;     /* 累计采样拍数，1 kHz 下每秒 +1000 */
extern volatile uint8_t  INS_Err;        /* 位标志：1=ID 不对 2=标定时有抖动 4=SPI 异常 */

extern volatile int16_t  INS_AccRaw[3];  /* 加速度计原始值 */
extern volatile int16_t  INS_GyroRaw[3]; /* 陀螺原始值 */
extern volatile float    INS_Acc[3];     /* m/s^2 */
extern volatile float    INS_Gyro[3];    /* rad/s，已去零偏、已低通 */
extern volatile float    INS_Bias[3];    /* 零偏（rad/s） */

/* ---------------- 调试：寄存器快照（0x00~0x0F） ---------------- */
extern volatile uint8_t  INS_AccDump[16];   /* [0] 应为 0x1E */
extern volatile uint8_t  INS_GyroDump[16];  /* [0] 应为 0x0F */

/* ---------------- 接口 ---------------- */
void INS_Init(void);        /* 初始化 BMI088，失败也会置 INS_Err 并返回 */
void INS_Task(void);        /* 1 kHz 调用一次 */
void INS_ResetBias(void);   /* 重新开始零偏标定（静止时调用） */

#ifdef __cplusplus
}
#endif

#endif /* __INS_H__ */
