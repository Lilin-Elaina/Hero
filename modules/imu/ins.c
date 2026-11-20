/**
  ******************************************************************************
  * @file    ins.c
  * @brief   BMI088 驱动（SPI2 + 双片选），只做「读角速度」这一件事
  *
  *  片上资源：
  *    SPI2  : PB13=SCK  PC2_C=MISO  PC1=MOSI   （CPOL=High, CPHA=2Edge, 4.125MHz）
  *    CS    : PC0 = 加速度计,  PC3 = 陀螺      （低有效）
  ******************************************************************************
  */

#include "ins.h"
#include "bsp_time.h"
#include "main.h"
#include "bsp_spi.h"
#include <string.h>
#include <math.h>

/* ======================= 引脚 =======================
 * 片选/模拟开关等片上细节已下沉到 bsp/bsp_spi.c（2026-11-19 P2d），
 * 这里只保留"用哪个器件"的语义。 */

/* Dev_Id: 0 = 加速度计, 1 = 陀螺 */
#define DEV_ACC   0u
#define DEV_GYRO  1u

/* ======================= 寄存器 ======================= */
/* 加速度计 */
#define ACC_CHIP_ID     0x00u
#define ACC_DATA        0x12u
#define ACC_CONF        0x40u
#define ACC_RANGE       0x41u
#define ACC_PWR_CONF    0x7Cu
#define ACC_PWR_CTRL    0x7Du
#define ACC_SOFTRESET   0x7Eu

/* 陀螺 */
#define GYRO_CHIP_ID    0x00u
#define GYRO_RATE_DATA  0x02u
#define GYRO_RANGE      0x0Fu
#define GYRO_BANDWIDTH  0x10u
#define GYRO_LPM1       0x11u
#define GYRO_SOFTRESET  0x14u

#define ACC_CHIP_ID_VAL   0x1Eu
#define GYRO_CHIP_ID_VAL  0x0Fu
#define BMI_SOFTRESET_VAL 0xB6u

/* ======================= 量程 ======================= */
/* 加速度 ±6g */
#define INS_ACC_LSB     (6.0f * 9.80665f / 32768.0f)
/* 陀螺 ±2000 dps */
#define INS_GYRO_LSB    (2000.0f * 0.0174532925f / 32768.0f)

/* 零偏标定拍数（1 kHz 下 2 s） */
#define INS_CAL_SAMPLES   2000u
/* 标定期间角速度超此值判为「动过了」，结果不可信 */
#define INS_CAL_MAX_DPS   5.0f
/* 一阶低通系数：0.333 ≈ 100 Hz @1 kHz */
#define INS_GYRO_LPF      0.3333f

/* ======================= 全局变量 ======================= */
volatile uint8_t  INS_AccId;
volatile uint8_t  INS_GyroId;
volatile uint8_t  INS_Ready;
volatile uint8_t  INS_CalDone;
volatile uint16_t INS_CalCnt;
volatile uint32_t INS_Sample;
volatile uint8_t  INS_Err;

volatile int16_t  INS_AccRaw[3];
volatile int16_t  INS_GyroRaw[3];
volatile float    INS_Acc[3];
volatile float    INS_Gyro[3];
volatile float    INS_Bias[3];

/* 调试用：两个芯片 0x00~0x0F 寄存器快照，用于判断 SPI 是位错位还是字节错位 */
volatile uint8_t  INS_AccDump[16];
volatile uint8_t  INS_GyroDump[16];

static float S_Sum[3];      /* 标定累加 */
static uint8_t S_Inited;

/* ======================= Mahony 姿态解算 =======================
 * 目的：拿到「世界垂直轴」上的角速度。板子竖直绑在枪架上，体轴 x 指上，
 * pitch 一动 x 轴就离开竖直，-Gyro[0] 不再是世界偏航角速度，必须用重力
 * 方向把陀螺投影到竖直轴上：ω_world = ω·v，v = 世界“上”在体轴系的投影。
 * 无磁力计 → 偏航角靠陀螺积分，零偏偏移会让 INS_WorldYaw 缓慢漂移（正常）。
 */
#define MAHONY_KP        10.0f
#define MAHONY_KI        0.008f
#define MAHONY_HALFT     0.0005f      /* 1 kHz 的半个采样周期 */
#define MAHONY_DT        0.001f
#define MAHONY_ACC_GATE  1.5f         /* |a| 偏离 1g 超此值(m/s^2) → 重力修正降到 10% */
#define MAHONY_WARMUP    1000u        /* 收敛 1 s 后才对外发布 */
/* 世界垂直轴 = SIGN·(ω·v)，v = 世界“上”在体轴系的投影。
 * 判据：静止水平时它必须等于既有约定 ω_yaw = -Gyro[0]。
 * 板子 x 轴朝下（所以当年才要那个负号）→ a_x≈-9.8 → v≈(-1,0,0)
 * → Gyro_Rad·v = -g0 → 要再取负一次才是 -g0，故 SIGN = +1。 */
#define INS_WORLD_YAW_SIGN  (+1.0f)

volatile float   INS_WorldYawRate;
volatile float   INS_WorldYaw;
volatile float   INS_RollDeg;
volatile float   INS_PitchDeg;
volatile uint8_t INS_AhrsReady;

static float    S_Q0 = 1.0f, S_Q1 = 0.0f, S_Q2 = 0.0f, S_Q3 = 0.0f;
static float    S_Exi, S_Eyi, S_Ezi;
static uint16_t S_Ahrs_Cnt;

static void ins_ahrs_step(float Gyro_X, float Gyro_Y, float Gyro_Z, float Acc_X, float Acc_Y, float Acc_Z)
{
    float Vec_Norm = sqrtf(Acc_X * Acc_X + Acc_Y * Acc_Y + Acc_Z * Acc_Z);
    float Up_X, Up_Y, Up_Z, Err_X, Err_Y, Err_Z, Kp_Gain;
    float Q0_Sq, Q0_Q1, Q0_Q2, Q1_Sq, Q1_Q3, Q2_Sq, Q2_Q3, Q3_Sq;
    float Q0_Old, Q1_Old, Q2_Old, Q3_Old;
    float Gyro_Raw_X = Gyro_X, Gyro_Raw_Y = Gyro_Y, Gyro_Raw_Z = Gyro_Z;

    if (Vec_Norm < 1.0f) {                    /* 数据无效，本拍跳过 */
        return;
    }

    Acc_X /= Vec_Norm;
    Acc_Y /= Vec_Norm;
    Acc_Z /= Vec_Norm;

    Q0_Sq = S_Q0 * S_Q0;
    Q0_Q1 = S_Q0 * S_Q1;
    Q0_Q2 = S_Q0 * S_Q2;
    Q1_Sq = S_Q1 * S_Q1;
    Q1_Q3 = S_Q1 * S_Q3;
    Q2_Sq = S_Q2 * S_Q2;
    Q2_Q3 = S_Q2 * S_Q3;
    Q3_Sq = S_Q3 * S_Q3;

    /* 估计的重力方向（世界“上”在体轴系） */
    Up_X = 2.0f * (Q1_Q3 - Q0_Q2);
    Up_Y = 2.0f * (Q0_Q1 + Q2_Q3);
    Up_Z = Q0_Sq - Q1_Sq - Q2_Sq + Q3_Sq;

    /* 实测重力与估计重力的叉积 = 姿态误差 */
    Err_X = Acc_Y * Up_Z - Acc_Z * Up_Y;
    Err_Y = Acc_Z * Up_X - Acc_X * Up_Z;
    Err_Z = Acc_X * Up_Y - Acc_Y * Up_X;

    /* 机体线加速度大时加速度计不可信，削弱修正，避免把姿态带偏 */
    Kp_Gain = MAHONY_KP;
    if (fabsf(Vec_Norm - 9.80665f) > MAHONY_ACC_GATE) {
        Kp_Gain *= 0.1f;
    }

    S_Exi += Err_X * (MAHONY_KI * MAHONY_DT);
    S_Eyi += Err_Y * (MAHONY_KI * MAHONY_DT);
    S_Ezi += Err_Z * (MAHONY_KI * MAHONY_DT);

    Gyro_X += Kp_Gain * Err_X + S_Exi;
    Gyro_Y += Kp_Gain * Err_Y + S_Eyi;
    Gyro_Z += Kp_Gain * Err_Z + S_Ezi;

    /* 四元数积分 */
    Q0_Old = S_Q0;
    Q1_Old = S_Q1;
    Q2_Old = S_Q2;
    Q3_Old = S_Q3;
    S_Q0 += (-Q1_Old * Gyro_X - Q2_Old * Gyro_Y - Q3_Old * Gyro_Z) * MAHONY_HALFT;
    S_Q1 += ( Q0_Old * Gyro_X + Q2_Old * Gyro_Z - Q3_Old * Gyro_Y) * MAHONY_HALFT;
    S_Q2 += ( Q0_Old * Gyro_Y - Q1_Old * Gyro_Z + Q3_Old * Gyro_X) * MAHONY_HALFT;
    S_Q3 += ( Q0_Old * Gyro_Z + Q1_Old * Gyro_Y - Q2_Old * Gyro_X) * MAHONY_HALFT;

    /* 归一化 */
    Vec_Norm = sqrtf(S_Q0 * S_Q0 + S_Q1 * S_Q1 + S_Q2 * S_Q2 + S_Q3 * S_Q3);
    if (Vec_Norm < 1e-6f) {
        S_Q0 = 1.0f; S_Q1 = 0.0f; S_Q2 = 0.0f; S_Q3 = 0.0f;
        return;
    }
    S_Q0 /= Vec_Norm;
    S_Q1 /= Vec_Norm;
    S_Q2 /= Vec_Norm;
    S_Q3 /= Vec_Norm;

    /* 用更新后的姿态重算竖直轴，把实测角速度投影上去（倾斜补偿的核心） */
    Q0_Sq = S_Q0 * S_Q0;
    Q0_Q1 = S_Q0 * S_Q1;
    Q0_Q2 = S_Q0 * S_Q2;
    Q1_Sq = S_Q1 * S_Q1;
    Q1_Q3 = S_Q1 * S_Q3;
    Q2_Sq = S_Q2 * S_Q2;
    Q2_Q3 = S_Q2 * S_Q3;
    Q3_Sq = S_Q3 * S_Q3;
    Up_X = 2.0f * (Q1_Q3 - Q0_Q2);
    Up_Y = 2.0f * (Q0_Q1 + Q2_Q3);
    Up_Z = Q0_Sq - Q1_Sq - Q2_Sq + Q3_Sq;

    /* 负号是为了和既有约定 ω_yaw = -INS_Gyro[0] 对齐（静止水平时两者相等） */
    INS_WorldYawRate = INS_WORLD_YAW_SIGN * (Gyro_Raw_X * Up_X + Gyro_Raw_Y * Up_Y + Gyro_Raw_Z * Up_Z) * 57.2957795f;

    if (S_Ahrs_Cnt < MAHONY_WARMUP) {
        S_Ahrs_Cnt++;
        if (S_Ahrs_Cnt == MAHONY_WARMUP) {
            INS_AhrsReady = 1u;
        }
        return;                     /* 收敛期内只跑姿态，不发布积分 */
    }

    INS_WorldYaw += INS_WorldYawRate * MAHONY_DT;
    if (INS_WorldYaw >  36000.0f) INS_WorldYaw -= 72000.0f;
    if (INS_WorldYaw < -36000.0f) INS_WorldYaw += 72000.0f;

    /* 调试：传感器体系下的欧拉角 */
    INS_RollDeg  = atan2f(2.0f * (Q2_Q3 + Q0_Q1), 1.0f - 2.0f * (Q1_Sq + Q2_Sq)) * 57.2957795f;
    INS_PitchDeg = asinf(2.0f * (Q0_Q2 - Q1_Q3)) * 57.2957795f;
}

/* ======================= 底层读写 ======================= */

static void ins_cs(uint8_t Dev_Id, uint8_t Cs_Low)
{
    BSP_IMU_Cs((Dev_Id == DEV_ACC) ? BSP_IMU_DEV_ACC : BSP_IMU_DEV_GYRO, Cs_Low);
}

/* 读寄存器时要丢弃的哑字节个数（上电后由 ins_detect_skip() 实测校准）
 * 2026-11-13：实测两个芯片都比原先假设的少 1 个哑字节——
 *   原先 acc=2 / gyro=1，读回 ID = 0x23 / 0x22，且读 0x00~0x0F 时
 *   数据整体错后一个寄存器（陀螺的 0x02~0x07 数据出现在 dump[1..6]），
 *   说明多丢了 1 个字节。默认改成 acc=1 / gyro=0，上电再自动确认。 */
static uint8_t S_Skip_Acc  = 1u;
static uint8_t S_Skip_Gyro = 0u;

/**
  * @brief  用指定的哑字节数读 1 个寄存器（探测/校验用）
  */
static uint8_t ins_read_at(uint8_t Dev_Id, uint8_t Skip_Bytes, uint8_t Reg_Addr)
{
    uint8_t  Tx_Buf[4] = {0};
    uint8_t  Rx_Buf[4] = {0};
    uint16_t Xfer_Len = (uint16_t)(1u + Skip_Bytes + 1u);

    Tx_Buf[0] = (uint8_t)(Reg_Addr | 0x80u);          /* bit7=1 → 读 */

    ins_cs(Dev_Id, 1);
    (void)BSP_IMU_Transfer(Tx_Buf, Rx_Buf, Xfer_Len);
    ins_cs(Dev_Id, 0);

    return Rx_Buf[1u + Skip_Bytes];
}

/**
  * @brief  读寄存器（自动处理 SPI 的「空字节」）
  * @note   哑字节个数取 S_Skip_Acc / S_Skip_Gyro（见上方说明）
  */
static uint8_t ins_read(uint8_t Dev_Id, uint8_t Reg_Addr, uint8_t *Out_Buf, uint16_t Out_Len)
{
    uint8_t  Tx_Buf[24];
    uint8_t  Rx_Buf[24];
    uint16_t Skip_Bytes = (Dev_Id == DEV_ACC) ? S_Skip_Acc : S_Skip_Gyro;
    uint16_t Xfer_Len    = (uint16_t)(1u + Skip_Bytes + Out_Len);
    uint16_t Idx;

    if (Xfer_Len > sizeof(Tx_Buf)) return 0u;

    memset(Tx_Buf, 0, sizeof(Tx_Buf));
    memset(Rx_Buf, 0, sizeof(Rx_Buf));
    Tx_Buf[0] = (uint8_t)(Reg_Addr | 0x80u);          /* bit7=1 → 读 */

    ins_cs(Dev_Id, 1);
    uint8_t Spi_Ok = BSP_IMU_Transfer(Tx_Buf, Rx_Buf, Xfer_Len);
    ins_cs(Dev_Id, 0);

    if (Spi_Ok == 0u) return 0u;                 /* SPI 失败：数据不可信 */

    for (Idx = 0; Idx < Out_Len; Idx++) {
        Out_Buf[Idx] = Rx_Buf[1u + Skip_Bytes + Idx];
    }
    return 1u;
}

static void ins_write(uint8_t Dev_Id, uint8_t Reg_Addr, uint8_t Reg_Val)
{
    uint8_t Tx_Buf[2];
    Tx_Buf[0] = (uint8_t)(Reg_Addr & 0x7Fu);          /* bit7=0 → 写 */
    Tx_Buf[1] = Reg_Val;

    ins_cs(Dev_Id, 1);
    (void)BSP_IMU_Send(Tx_Buf, 2);
    ins_cs(Dev_Id, 0);
}

/* ======================= 初始化 ======================= */

/**
  * @brief  抓取两个芯片 0x00~0x0F 的寄存器快照（只在初始化后调用一次）
  */
static void ins_dump(void)
{
    uint8_t Idx;

    /* [0..2] 是偏移探测结果、[3] 是选中的哑字节数，这里只覆盖 [4..15] */
    for (Idx = 4u; Idx < 16u; Idx++) {
        ins_read(DEV_ACC,  Idx, (uint8_t *)&INS_AccDump[Idx],  1u);
        ins_read(DEV_GYRO, Idx, (uint8_t *)&INS_GyroDump[Idx], 1u);
    }
}

/**
  * @brief  探测 SPI 读偏移（要丢几个哑字节）
  * @note   对每个芯片用 Skip_Bytes = 0/1/2 各读一次 CHIP_ID：
  *           INS_xxxDump[0..2] = 三次读到的值
  *           INS_xxxDump[3]    = 最终选中的 Skip_Bytes
  *         谁先读到正确 ID（acc 0x1E / gyro 0x0F）就用谁；
  *         一个都对不上就退回默认 acc=1 / gyro=0。
  */
static void ins_detect_skip(void)
{
    uint8_t Cand_Skip;

    /* 上电第一次读还在 I2C→SPI 切换过程中，先空跑一轮把两片切到 SPI */
    for (Cand_Skip = 0; Cand_Skip < 3u; Cand_Skip++) {
        (void)ins_read_at(DEV_ACC,  Cand_Skip, ACC_CHIP_ID);
        (void)ins_read_at(DEV_GYRO, Cand_Skip, GYRO_CHIP_ID);
    }

    for (Cand_Skip = 0; Cand_Skip < 3u; Cand_Skip++) {
        INS_AccDump[Cand_Skip]  = ins_read_at(DEV_ACC,  Cand_Skip, ACC_CHIP_ID);
        INS_GyroDump[Cand_Skip] = ins_read_at(DEV_GYRO, Cand_Skip, GYRO_CHIP_ID);
    }

    S_Skip_Acc  = 0xFFu;
    S_Skip_Gyro = 0xFFu;

    for (Cand_Skip = 0; Cand_Skip < 3u; Cand_Skip++) {
        if ((INS_AccDump[Cand_Skip]  == ACC_CHIP_ID_VAL)  && (S_Skip_Acc  == 0xFFu)) S_Skip_Acc  = Cand_Skip;
        if ((INS_GyroDump[Cand_Skip] == GYRO_CHIP_ID_VAL) && (S_Skip_Gyro == 0xFFu)) S_Skip_Gyro = Cand_Skip;
    }

    if (S_Skip_Acc  == 0xFFu) S_Skip_Acc  = 1u;
    if (S_Skip_Gyro == 0xFFu) S_Skip_Gyro = 0u;

    INS_AccDump[3]  = S_Skip_Acc;
    INS_GyroDump[3] = S_Skip_Gyro;
}

/**
  * @brief  片选脚初始化 + H7 模拟开关
  * @note   PC0/PC3 在 CubeMX 里没配成输出（gpio.c 只管了 PD2），
  *         不在这里配的话片选是悬空的，芯片永远不会被选中 → 读回 0x00。
  */
/* 当前被 INS_Init 里的“临时停用”注释掉；保留函数避免以后要用时重写 */
static void __attribute__((unused)) ins_gpio_init(void)
{
    /* 片选 GPIO + H7 模拟开关：实现在 bsp_spi.c */
    BSP_IMU_Init();
}

static void ins_acc_init(void)
{
    uint8_t Chip_Id = 0;

    /* 上电默认 I2C 模式：CS 拉高再拉低，产生上升沿后切到 SPI */
    BSP_IMU_Cs(BSP_IMU_DEV_ACC, 0);
    BSP_DelayMs(2);

    /* 第一次读多半读到 0xFF/0x00（正在切模式），读两次更稳 */
    ins_read(DEV_ACC, ACC_CHIP_ID, &Chip_Id, 1);
    BSP_DelayMs(1);
    ins_read(DEV_ACC, ACC_CHIP_ID, &Chip_Id, 1);
    INS_AccId = Chip_Id;

    if (Chip_Id != ACC_CHIP_ID_VAL) {
        INS_Err |= 0x01u;
        return;
    }

    ins_write(DEV_ACC, ACC_SOFTRESET, BMI_SOFTRESET_VAL);
    BSP_DelayMs(50);

    /* 复位后要重新建立 SPI 通信 */
    ins_read(DEV_ACC, ACC_CHIP_ID, &Chip_Id, 1);
    BSP_DelayMs(1);
    ins_read(DEV_ACC, ACC_CHIP_ID, &Chip_Id, 1);
    INS_AccId = Chip_Id;

    if (Chip_Id != ACC_CHIP_ID_VAL) {
        INS_Err |= 0x01u;
        return;
    }

    ins_write(DEV_ACC, ACC_PWR_CTRL, 0x04u);   /* 打开加速度计 */
    BSP_DelayMs(5);
    ins_write(DEV_ACC, ACC_PWR_CONF, 0x00u);   /* active 模式 */

    ins_write(DEV_ACC, ACC_CONF,  0xACu);      /* 正常模式 / 1600Hz / 带宽 = ODR/2 */
    ins_write(DEV_ACC, ACC_RANGE, 0x01u);      /* ±6g */
    BSP_DelayMs(10);
}

static void ins_gyro_init(void)
{
    uint8_t Chip_Id = 0;

    BSP_IMU_Cs(BSP_IMU_DEV_GYRO, 0);
    BSP_DelayMs(2);

    ins_read(DEV_GYRO, GYRO_CHIP_ID, &Chip_Id, 1);
    INS_GyroId = Chip_Id;

    if (Chip_Id != GYRO_CHIP_ID_VAL) {
        INS_Err |= 0x01u;
        return;
    }

    ins_write(DEV_GYRO, GYRO_SOFTRESET, BMI_SOFTRESET_VAL);
    BSP_DelayMs(30);

    ins_read(DEV_GYRO, GYRO_CHIP_ID, &Chip_Id, 1);
    INS_GyroId = Chip_Id;

    if (Chip_Id != GYRO_CHIP_ID_VAL) {
        INS_Err |= 0x01u;
        return;
    }

    ins_write(DEV_GYRO, GYRO_RANGE,     0x00u);  /* ±2000 dps */
    ins_write(DEV_GYRO, GYRO_BANDWIDTH, 0x02u);  /* 1000Hz ODR / 116Hz 滤波 */
    ins_write(DEV_GYRO, GYRO_LPM1,      0x00u);  /* 正常模式 */
    BSP_DelayMs(5);
}

void INS_Init(void)
{
    uint8_t Idx;

    INS_AccId   = 0;
    INS_GyroId  = 0;
    INS_Ready   = 0;
    INS_CalDone = 0;
    INS_CalCnt  = 0;
    INS_Sample  = 0;
    INS_Err     = 0;
    S_Inited    = 0;

    for (Idx = 0; Idx < 3; Idx++) {
        INS_Bias[Idx]  = 0.0f;
        INS_Gyro[Idx]  = 0.0f;
        INS_Acc[Idx]   = 0.0f;
        S_Sum[Idx]     = 0.0f;
    }

    BSP_IMU_Cs(BSP_IMU_DEV_ACC, 0);
    BSP_IMU_Cs(BSP_IMU_DEV_GYRO, 0);
    /* ins_gpio_init();   临时停用：验证 PC0/PC3 是否与 CAN2 收发器共用引脚 */
    BSP_DelayMs(10);

    ins_detect_skip();          /* 先量出 SPI 哑字节个数，再按它初始化 */
    ins_acc_init();
    ins_gyro_init();
    ins_dump();

    /* 两个 ID 都对上才算就绪 */
    if ((INS_Err & 0x01u) == 0u) {
        INS_Ready = 1;
        S_Inited  = 1;
    }
}

void INS_ResetBias(void)
{
    uint8_t Idx;
    for (Idx = 0; Idx < 3; Idx++) {
        S_Sum[Idx]    = 0.0f;
        INS_Bias[Idx] = 0.0f;
        INS_Gyro[Idx] = 0.0f;
    }
    INS_CalCnt  = 0;
    INS_CalDone = 0;
}

/* ======================= 周期任务 ======================= */

void INS_Task(void)
{
    uint8_t  Raw_Buf[6];
    uint8_t  Idx;
    float    Gyro_Rad[3];
    float    Max_Rate_Dps = 0.0f;

    if (S_Inited == 0u || INS_Ready == 0u) {
        return;
    }

    /* ---- 陀螺：0x02 ~ 0x07 ---- */
    if (ins_read(DEV_GYRO, GYRO_RATE_DATA, Raw_Buf, 6) == 0u) {
        INS_Err |= 0x04u;        /* SPI 读失败：本拍数据不可信，直接丢弃 */
        return;
    }
    INS_GyroRaw[0] = (int16_t)(((uint16_t)Raw_Buf[1] << 8) | Raw_Buf[0]);
    INS_GyroRaw[1] = (int16_t)(((uint16_t)Raw_Buf[3] << 8) | Raw_Buf[2]);
    INS_GyroRaw[2] = (int16_t)(((uint16_t)Raw_Buf[5] << 8) | Raw_Buf[4]);

    for (Idx = 0; Idx < 3; Idx++) {
        Gyro_Rad[Idx] = (float)INS_GyroRaw[Idx] * INS_GYRO_LSB;      /* rad/s */
        if (Gyro_Rad[Idx] > Max_Rate_Dps)  Max_Rate_Dps = Gyro_Rad[Idx];
        if (-Gyro_Rad[Idx] > Max_Rate_Dps) Max_Rate_Dps = -Gyro_Rad[Idx];
    }

    INS_Sample++;

    /* ---- 零偏标定 ---- */
    if (INS_CalDone == 0u) {
        for (Idx = 0; Idx < 3; Idx++) {
            S_Sum[Idx] += Gyro_Rad[Idx];
            INS_Gyro[Idx] = 0.0f;
        }
        INS_CalCnt++;

        /* 标定过程中动过 → 作废重来，不能带着动过的样本收零偏 */
        if (Max_Rate_Dps > (INS_CAL_MAX_DPS * 0.0174532925f)) {
            INS_Err |= 0x02u;
            for (Idx = 0; Idx < 3; Idx++) {
                S_Sum[Idx] = 0.0f;
            }
            INS_CalCnt = 0;
            return;
        }

        if (INS_CalCnt >= INS_CAL_SAMPLES) {
            for (Idx = 0; Idx < 3; Idx++) {
                INS_Bias[Idx] = S_Sum[Idx] / (float)INS_CAL_SAMPLES;
            }
            INS_CalDone = 1;
        }
        return;
    }

    /* ---- 去零偏 + 一阶低通 ---- */
    for (Idx = 0; Idx < 3; Idx++) {
        float v = Gyro_Rad[Idx] - INS_Bias[Idx];
        INS_Gyro[Idx] += (v - INS_Gyro[Idx]) * INS_GYRO_LPF;
    }

    /* ---- 加速度计：0x12 ~ 0x17 ---- */
    if (ins_read(DEV_ACC, ACC_DATA, Raw_Buf, 6) == 0u) {
        INS_Err |= 0x04u;        /* SPI 读失败：本次六轴样本不完整，不发布 */
        return;
    }
    INS_AccRaw[0] = (int16_t)(((uint16_t)Raw_Buf[1] << 8) | Raw_Buf[0]);
    INS_AccRaw[1] = (int16_t)(((uint16_t)Raw_Buf[3] << 8) | Raw_Buf[2]);
    INS_AccRaw[2] = (int16_t)(((uint16_t)Raw_Buf[5] << 8) | Raw_Buf[4]);

    for (Idx = 0; Idx < 3; Idx++) {
        INS_Acc[Idx] = (float)INS_AccRaw[Idx] * INS_ACC_LSB;
    }

    /* ---- Mahony：世界偏航角速度 / 世界偏航角（倾斜补偿） ---- */
    ins_ahrs_step(INS_Gyro[0], INS_Gyro[1], INS_Gyro[2],
                  INS_Acc[0],  INS_Acc[1],  INS_Acc[2]);
}
