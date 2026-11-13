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
#include "spi.h"
#include "main.h"
#include <string.h>

/* ======================= 引脚 ======================= */
#define INS_CS_PORT        GPIOC
#define INS_CS_ACC_PIN     GPIO_PIN_0
#define INS_CS_GYRO_PIN    GPIO_PIN_3

#define INS_CS_ACC_LOW()   HAL_GPIO_WritePin(INS_CS_PORT, INS_CS_ACC_PIN,  GPIO_PIN_RESET)
#define INS_CS_ACC_HIGH()  HAL_GPIO_WritePin(INS_CS_PORT, INS_CS_ACC_PIN,  GPIO_PIN_SET)
#define INS_CS_GYRO_LOW()  HAL_GPIO_WritePin(INS_CS_PORT, INS_CS_GYRO_PIN, GPIO_PIN_RESET)
#define INS_CS_GYRO_HIGH() HAL_GPIO_WritePin(INS_CS_PORT, INS_CS_GYRO_PIN, GPIO_PIN_SET)

/* dev: 0 = 加速度计, 1 = 陀螺 */
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

static float s_sum[3];      /* 标定累加 */
static uint8_t s_inited;

/* ======================= 底层读写 ======================= */

static void ins_cs(uint8_t dev, uint8_t low)
{
    if (dev == DEV_ACC) {
        if (low) INS_CS_ACC_LOW(); else INS_CS_ACC_HIGH();
    } else {
        if (low) INS_CS_GYRO_LOW(); else INS_CS_GYRO_HIGH();
    }
}

/* 读寄存器时要丢弃的哑字节个数（上电后由 ins_detect_skip() 实测校准）
 * 2026-11-13：实测两个芯片都比原先假设的少 1 个哑字节——
 *   原先 acc=2 / gyro=1，读回 ID = 0x23 / 0x22，且读 0x00~0x0F 时
 *   数据整体错后一个寄存器（陀螺的 0x02~0x07 数据出现在 dump[1..6]），
 *   说明多丢了 1 个字节。默认改成 acc=1 / gyro=0，上电再自动确认。 */
static uint8_t s_skip_acc  = 1u;
static uint8_t s_skip_gyro = 0u;

/**
  * @brief  用指定的哑字节数读 1 个寄存器（探测/校验用）
  */
static uint8_t ins_read_at(uint8_t dev, uint8_t skip, uint8_t reg)
{
    uint8_t  tx[4] = {0};
    uint8_t  rx[4] = {0};
    uint16_t n = (uint16_t)(1u + skip + 1u);

    tx[0] = (uint8_t)(reg | 0x80u);          /* bit7=1 → 读 */

    ins_cs(dev, 1);
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, n, 50);
    ins_cs(dev, 0);

    return rx[1u + skip];
}

/**
  * @brief  读寄存器（自动处理 SPI 的「空字节」）
  * @note   哑字节个数取 s_skip_acc / s_skip_gyro（见上方说明）
  */
static uint8_t ins_read(uint8_t dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t  tx[24];
    uint8_t  rx[24];
    uint16_t skip = (dev == DEV_ACC) ? s_skip_acc : s_skip_gyro;
    uint16_t n    = (uint16_t)(1u + skip + len);
    uint16_t i;

    if (n > sizeof(tx)) return 0u;

    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));
    tx[0] = (uint8_t)(reg | 0x80u);          /* bit7=1 → 读 */

    ins_cs(dev, 1);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi2, tx, rx, n, 50);
    ins_cs(dev, 0);

    if (st != HAL_OK) return 0u;             /* SPI 失败：数据不可信 */

    for (i = 0; i < len; i++) {
        buf[i] = rx[1u + skip + i];
    }
    return 1u;
}

static void ins_write(uint8_t dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    tx[0] = (uint8_t)(reg & 0x7Fu);          /* bit7=0 → 写 */
    tx[1] = val;

    ins_cs(dev, 1);
    HAL_SPI_Transmit(&hspi2, tx, 2, 20);
    ins_cs(dev, 0);
}

/* ======================= 初始化 ======================= */

/**
  * @brief  抓取两个芯片 0x00~0x0F 的寄存器快照（只在初始化后调用一次）
  */
static void ins_dump(void)
{
    uint8_t i;

    /* [0..2] 是偏移探测结果、[3] 是选中的哑字节数，这里只覆盖 [4..15] */
    for (i = 4u; i < 16u; i++) {
        ins_read(DEV_ACC,  i, (uint8_t *)&INS_AccDump[i],  1u);
        ins_read(DEV_GYRO, i, (uint8_t *)&INS_GyroDump[i], 1u);
    }
}

/**
  * @brief  探测 SPI 读偏移（要丢几个哑字节）
  * @note   对每个芯片用 skip = 0/1/2 各读一次 CHIP_ID：
  *           INS_xxxDump[0..2] = 三次读到的值
  *           INS_xxxDump[3]    = 最终选中的 skip
  *         谁先读到正确 ID（acc 0x1E / gyro 0x0F）就用谁；
  *         一个都对不上就退回默认 acc=1 / gyro=0。
  */
static void ins_detect_skip(void)
{
    uint8_t k;

    /* 上电第一次读还在 I2C→SPI 切换过程中，先空跑一轮把两片切到 SPI */
    for (k = 0; k < 3u; k++) {
        (void)ins_read_at(DEV_ACC,  k, ACC_CHIP_ID);
        (void)ins_read_at(DEV_GYRO, k, GYRO_CHIP_ID);
    }

    for (k = 0; k < 3u; k++) {
        INS_AccDump[k]  = ins_read_at(DEV_ACC,  k, ACC_CHIP_ID);
        INS_GyroDump[k] = ins_read_at(DEV_GYRO, k, GYRO_CHIP_ID);
    }

    s_skip_acc  = 0xFFu;
    s_skip_gyro = 0xFFu;

    for (k = 0; k < 3u; k++) {
        if ((INS_AccDump[k]  == ACC_CHIP_ID_VAL)  && (s_skip_acc  == 0xFFu)) s_skip_acc  = k;
        if ((INS_GyroDump[k] == GYRO_CHIP_ID_VAL) && (s_skip_gyro == 0xFFu)) s_skip_gyro = k;
    }

    if (s_skip_acc  == 0xFFu) s_skip_acc  = 1u;
    if (s_skip_gyro == 0xFFu) s_skip_gyro = 0u;

    INS_AccDump[3]  = s_skip_acc;
    INS_GyroDump[3] = s_skip_gyro;
}

/**
  * @brief  片选脚初始化 + H7 模拟开关
  * @note   PC0/PC3 在 CubeMX 里没配成输出（gpio.c 只管了 PD2），
  *         不在这里配的话片选是悬空的，芯片永远不会被选中 → 读回 0x00。
  */
static void ins_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* 先把电平置高（未选中），再切成输出，避免切换瞬间误选通 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_3, GPIO_PIN_SET);

    g.Pin   = GPIO_PIN_0 | GPIO_PIN_3;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &g);

    /* H7 的 PC2_C/PC3_C 与 PC2/PC3 之间有模拟开关，不闭合则引脚不通 */
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);
}

static void ins_acc_init(void)
{
    uint8_t id = 0;

    /* 上电默认 I2C 模式：CS 拉高再拉低，产生上升沿后切到 SPI */
    INS_CS_ACC_HIGH();
    HAL_Delay(2);

    /* 第一次读多半读到 0xFF/0x00（正在切模式），读两次更稳 */
    ins_read(DEV_ACC, ACC_CHIP_ID, &id, 1);
    HAL_Delay(1);
    ins_read(DEV_ACC, ACC_CHIP_ID, &id, 1);
    INS_AccId = id;

    if (id != ACC_CHIP_ID_VAL) {
        INS_Err |= 0x01u;
        return;
    }

    ins_write(DEV_ACC, ACC_SOFTRESET, BMI_SOFTRESET_VAL);
    HAL_Delay(50);

    /* 复位后要重新建立 SPI 通信 */
    ins_read(DEV_ACC, ACC_CHIP_ID, &id, 1);
    HAL_Delay(1);
    ins_read(DEV_ACC, ACC_CHIP_ID, &id, 1);
    INS_AccId = id;

    if (id != ACC_CHIP_ID_VAL) {
        INS_Err |= 0x01u;
        return;
    }

    ins_write(DEV_ACC, ACC_PWR_CTRL, 0x04u);   /* 打开加速度计 */
    HAL_Delay(5);
    ins_write(DEV_ACC, ACC_PWR_CONF, 0x00u);   /* active 模式 */

    ins_write(DEV_ACC, ACC_CONF,  0xACu);      /* 正常模式 / 1600Hz / 带宽 = ODR/2 */
    ins_write(DEV_ACC, ACC_RANGE, 0x01u);      /* ±6g */
    HAL_Delay(10);
}

static void ins_gyro_init(void)
{
    uint8_t id = 0;

    INS_CS_GYRO_HIGH();
    HAL_Delay(2);

    ins_read(DEV_GYRO, GYRO_CHIP_ID, &id, 1);
    INS_GyroId = id;

    if (id != GYRO_CHIP_ID_VAL) {
        INS_Err |= 0x01u;
        return;
    }

    ins_write(DEV_GYRO, GYRO_SOFTRESET, BMI_SOFTRESET_VAL);
    HAL_Delay(30);

    ins_read(DEV_GYRO, GYRO_CHIP_ID, &id, 1);
    INS_GyroId = id;

    if (id != GYRO_CHIP_ID_VAL) {
        INS_Err |= 0x01u;
        return;
    }

    ins_write(DEV_GYRO, GYRO_RANGE,     0x00u);  /* ±2000 dps */
    ins_write(DEV_GYRO, GYRO_BANDWIDTH, 0x02u);  /* 1000Hz ODR / 116Hz 滤波 */
    ins_write(DEV_GYRO, GYRO_LPM1,      0x00u);  /* 正常模式 */
    HAL_Delay(5);
}

void INS_Init(void)
{
    uint8_t i;

    INS_AccId   = 0;
    INS_GyroId  = 0;
    INS_Ready   = 0;
    INS_CalDone = 0;
    INS_CalCnt  = 0;
    INS_Sample  = 0;
    INS_Err     = 0;
    s_inited    = 0;

    for (i = 0; i < 3; i++) {
        INS_Bias[i]  = 0.0f;
        INS_Gyro[i]  = 0.0f;
        INS_Acc[i]   = 0.0f;
        s_sum[i]     = 0.0f;
    }

    INS_CS_ACC_HIGH();
    INS_CS_GYRO_HIGH();
    /* ins_gpio_init();   临时停用：验证 PC0/PC3 是否与 CAN2 收发器共用引脚 */
    HAL_Delay(10);

    ins_detect_skip();          /* 先量出 SPI 哑字节个数，再按它初始化 */
    ins_acc_init();
    ins_gyro_init();
    ins_dump();

    /* 两个 ID 都对上才算就绪 */
    if ((INS_Err & 0x01u) == 0u) {
        INS_Ready = 1;
        s_inited  = 1;
    }
}

void INS_ResetBias(void)
{
    uint8_t i;
    for (i = 0; i < 3; i++) {
        s_sum[i]    = 0.0f;
        INS_Bias[i] = 0.0f;
        INS_Gyro[i] = 0.0f;
    }
    INS_CalCnt  = 0;
    INS_CalDone = 0;
}

/* ======================= 周期任务 ======================= */

void INS_Task(void)
{
    uint8_t  b[6];
    uint8_t  i;
    float    g[3];
    float    max_dps = 0.0f;

    if (s_inited == 0u || INS_Ready == 0u) {
        return;
    }

    /* ---- 陀螺：0x02 ~ 0x07 ---- */
    if (ins_read(DEV_GYRO, GYRO_RATE_DATA, b, 6) == 0u) {
        INS_Err |= 0x04u;        /* SPI 读失败：本拍数据不可信，直接丢弃 */
        return;
    }
    INS_GyroRaw[0] = (int16_t)(((uint16_t)b[1] << 8) | b[0]);
    INS_GyroRaw[1] = (int16_t)(((uint16_t)b[3] << 8) | b[2]);
    INS_GyroRaw[2] = (int16_t)(((uint16_t)b[5] << 8) | b[4]);

    for (i = 0; i < 3; i++) {
        g[i] = (float)INS_GyroRaw[i] * INS_GYRO_LSB;      /* rad/s */
        if (g[i] > max_dps)  max_dps = g[i];
        if (-g[i] > max_dps) max_dps = -g[i];
    }

    INS_Sample++;

    /* ---- 零偏标定 ---- */
    if (INS_CalDone == 0u) {
        for (i = 0; i < 3; i++) {
            s_sum[i] += g[i];
            INS_Gyro[i] = 0.0f;
        }
        INS_CalCnt++;

        /* 标定过程中动过 → 作废重来，不能带着动过的样本收零偏 */
        if (max_dps > (INS_CAL_MAX_DPS * 0.0174532925f)) {
            INS_Err |= 0x02u;
            for (i = 0; i < 3; i++) {
                s_sum[i] = 0.0f;
            }
            INS_CalCnt = 0;
            return;
        }

        if (INS_CalCnt >= INS_CAL_SAMPLES) {
            for (i = 0; i < 3; i++) {
                INS_Bias[i] = s_sum[i] / (float)INS_CAL_SAMPLES;
            }
            INS_CalDone = 1;
        }
        return;
    }

    /* ---- 去零偏 + 一阶低通 ---- */
    for (i = 0; i < 3; i++) {
        float v = g[i] - INS_Bias[i];
        INS_Gyro[i] += (v - INS_Gyro[i]) * INS_GYRO_LPF;
    }

    /* ---- 加速度计：0x12 ~ 0x17 ---- */
    if (ins_read(DEV_ACC, ACC_DATA, b, 6) == 0u) {
        INS_Err |= 0x04u;        /* SPI 读失败：本次六轴样本不完整，不发布 */
        return;
    }
    INS_AccRaw[0] = (int16_t)(((uint16_t)b[1] << 8) | b[0]);
    INS_AccRaw[1] = (int16_t)(((uint16_t)b[3] << 8) | b[2]);
    INS_AccRaw[2] = (int16_t)(((uint16_t)b[5] << 8) | b[4]);

    for (i = 0; i < 3; i++) {
        INS_Acc[i] = (float)INS_AccRaw[i] * INS_ACC_LSB;
    }
}
