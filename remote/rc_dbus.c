/**
 * @file    rc_dbus.c
 * @brief   DT7 + DR16 遥控器 DBUS 接收与解析
 * @note    硬件：达妙 DM-MC-Board02 SBUS 口（官方例程走 UART5 / RX=PD2 AF8）
 *          协议：100000bps, 8数据位+偶校验, 2停止位, 每帧18字节
 */
#include "rc_dbus.h"
#include "usart.h"
#include "DJ_Motor.h"
#include "mecanum.h"
#include <string.h>

RC_TypeDef RC_Data = {0};

/* 最近一次 RC_Control_Task 算出的各轮归一化目标（调试用，可在调试器里看） */
float RC_WheelOut[MECANUM_WHEEL_NUM] = {0};

/* 使能条件持续的起始时刻：用于“拨杆刚到位也要等 100ms 才给电流”的软启动门 */
static uint32_t rc_enable_since = 0;

/* DMA 接收缓冲：2 帧容量
 *
 * ★ 必须放在 DMA 可访问的内存里：STM32H7 的 DTCM(0x20000000) 只有 Cortex-M7
 *   的 D-bus 能访问，DMA1/DMA2 往 DTCM 写会在第一个字节就报传输错误。
 *   本工程 .data/.bss 都在 DTCMRAM，所以这里显式指定 .dma_buffer 段，
 *   该段在 STM32H723xG_flash.ld 里被链接到 RAM_D1(0x24000000)。
 *   32 字节对齐是为了将来若打开 D-Cache 仍能安全做 cache 维护。 */
#if defined(__GNUC__)
static uint8_t dbus_buf[DBUS_FRAME_LEN * 2]
    __attribute__((section(".dma_buffer"), aligned(32)));
#else
static uint8_t dbus_buf[DBUS_FRAME_LEN * 2];
#endif

void RC_StartReceive(void)
{
    /* 上一帧若因溢出/噪声卡在 ORE，不清掉就再也收不到新数据 */
    __HAL_UART_CLEAR_OREFLAG(&huart5);

    /* DMA 普通模式，靠串口总线空闲（IDLE）判定一帧结束 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dbus_buf, sizeof(dbus_buf)) != HAL_OK)
    {
        /* 启动失败（上一次接收还没拆干净）：强制中止后重试一次，
         * 否则接收链会永久静默，表现为“遥控器完全没反应” */
        HAL_UART_AbortReceive(&huart5);
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dbus_buf, sizeof(dbus_buf));
    }

    /* 关掉半传输中断，避免回调被提前触发一次 */
    if (huart5.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
}

void RC_Init(void)
{
    memset(&RC_Data, 0, sizeof(RC_Data));
    RC_Data.online = 0;
    RC_Data.sw_swap = 1u;
    RC_StartReceive();
}

/* ------------------------------------------------------------------ */
/* 18 字节帧完整解析（与大疆官方 sbus_to_rc() 逐位一致）                */
/*   byte0~5  : 4 路摇杆×11bit + 2 路拨杆×2bit                         */
/*   byte6~13 : 鼠标 x/y/z + 左右键                                    */
/*   byte14~15: 键盘按键位图                                           */
/*   byte16~17: 左拨轮(11bit)                                          */
/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/* 拨杆位域自动判定（确定性，不需人工判读）                              */
/*   实测法：把 DT7 左拨杆停在非中位、右拨杆保持中位，持续 1 秒。         */
/*   这一秒内“稳定处于非中位”的那个位域 = 物理左拨杆 → 自动置 sw_swap。  */
/*   只在 sw_auto_window_ms 内开放，判定成功即永久锁定。                 */
/* ------------------------------------------------------------------ */
#define RC_SW_AUTO_HOLD_MS      1000    /* 条件需连续保持多少毫秒 */
#define RC_SW_AUTO_WINDOW_MS    3000    /* 上电后多久内允许自动判定 */

#define RC_OBS_NONE      0              /* 两个位域都在中位 */
#define RC_OBS_LO_MOVED  1              /* 只有 bit4~5 离开中位 */
#define RC_OBS_HI_MOVED  2              /* 只有 bit6~7 离开中位 */
#define RC_OBS_INVALID   3              /* 两个都离开中位 → 本段不计 */

void RC_SwitchAutoName(void)
{
    uint32_t now = HAL_GetTick();

    if (RC_Data.sw_auto_done)
    {
        return;                                     /* 已锁定，永不改写 */
    }

    /* 第一帧到达时开判定窗口（此前的上电时间不计入窗口） */
    if (RC_Data.sw_first_frame_ms == 0)
    {
        RC_Data.sw_first_frame_ms = now ? now : 1u;
        RC_Data.sw_obs_start_ms   = 0;
        RC_Data.sw_obs_last       = RC_OBS_NONE;
        RC_Data.sw_obs_hold       = 0;
    }

    if ((now - RC_Data.sw_first_frame_ms) > RC_SW_AUTO_WINDOW_MS)
    {
        return;                                     /* 窗口已过 → 保持默认，永久放弃 */
    }

    /* 本帧观测：必须“恰好一个位域非中位”才有信息量 */
    uint8_t lo_mid = (RC_Data.sw_lo2 == RC_SW_MID) ? 1u : 0u;
    uint8_t hi_mid = (RC_Data.sw_hi2 == RC_SW_MID) ? 1u : 0u;
    uint8_t obs;

    if (lo_mid && hi_mid)        obs = RC_OBS_NONE;
    else if (!lo_mid && hi_mid)  obs = RC_OBS_LO_MOVED;
    else if (lo_mid && !hi_mid)  obs = RC_OBS_HI_MOVED;
    else                         obs = RC_OBS_INVALID;

    if ((obs != RC_Data.sw_obs_last) || (obs == RC_OBS_NONE) || (obs == RC_OBS_INVALID))
    {
        /* 观测状态变了（或本段无信息量）→ 重新开始计时 */
        RC_Data.sw_obs_last     = obs;
        RC_Data.sw_obs_hold     = 0;
        RC_Data.sw_obs_start_ms = (obs == RC_OBS_LO_MOVED || obs == RC_OBS_HI_MOVED) ? now : 0u;
        return;
    }

    if (RC_Data.sw_obs_hold < 250u)
    {
        RC_Data.sw_obs_hold++;
    }

    if (((RC_Data.sw_obs_start_ms != 0u) ? (now - RC_Data.sw_obs_start_ms)
                                        : 0u) >= RC_SW_AUTO_HOLD_MS)
    {
        /* 连续 1 秒只有一方离开中位 → 判定完成并永久锁定 */
        RC_Data.sw_auto_result = (obs == RC_OBS_HI_MOVED) ? 1u : 0u;
        RC_Data.sw_swap        = RC_Data.sw_auto_result;
        RC_Data.sw_auto_done   = 1u;
    }
}

void RC_Decode(uint8_t *b)
{
    /* ---- 4 路摇杆 × 11bit，LSB 先 / 小端顺序打包 ---- */
    RC_Data.ch[0] = (uint16_t)(b[0]            | ((uint16_t)b[1] << 8))                 & 0x07FF;
    RC_Data.ch[1] = (uint16_t)((b[1] >> 3)     | ((uint16_t)b[2] << 5))                 & 0x07FF;
    RC_Data.ch[2] = (uint16_t)((b[2] >> 6)     | ((uint16_t)b[3] << 2) | ((uint16_t)b[4] << 10)) & 0x07FF;
    RC_Data.ch[3] = (uint16_t)((b[4] >> 1)     | ((uint16_t)b[5] << 7))                 & 0x07FF;

    /* ---- 2 路拨杆，各 2bit，放在 b[5] 的高 4 位 ----
     * 与官方 RC_ctrl_t.s[] 的对应关系（位运算不动，命名靠 sw_swap 调）：
     *   sw_lo2 = (b[5]>>4)&0x03       = 官方 s[0]（bit4~5）
     *   sw_hi2 = ((b[5]>>4)&0x0C)>>2  = 官方 s[1]（bit6~7）
     * 官方 s[0]/s[1] 哪个是物理左拨杆有版本分歧，用 sw_swap 一次性量准
     * （见 rc_dbus.h 的 ★验证法，位运算本身不用动）*/
    RC_Data.sw_lo2 = RC_SW_BITS_LO(b[5]);
    RC_Data.sw_hi2 = RC_SW_BITS_HI(b[5]);

    /* ---- 拨杆位域自动判定：必须在下面的命名映射之前调用，判定结果当帧生效 ----
     * 实测法（上电后 3 秒内做一次即可）：把左拨杆停在“上”或“下”，
     * 保持右拨杆在中位不动，持续约 1 秒 → sw_auto_done 变 1、sw_swap 自动定好。*/
    RC_SwitchAutoName();

    if (RC_Data.sw_swap)
    {
        RC_Data.s_left  = RC_Data.sw_hi2;
        RC_Data.s_right = RC_Data.sw_lo2;
    }
    else
    {
        RC_Data.s_left  = RC_Data.sw_lo2;
        RC_Data.s_right = RC_Data.sw_hi2;
    }

    /* ---- 鼠标 + 键盘（完整解出，保证整帧 18 字节全覆盖）----
     * 控制层暂不使用，但解出后可在调试器里核验通信正确性 */
    RC_Data.mouse.x        = (int16_t)(b[6]  | ((uint16_t)b[7]  << 8));
    RC_Data.mouse.y        = (int16_t)(b[8]  | ((uint16_t)b[9]  << 8));
    RC_Data.mouse.z        = (int16_t)((b[10] | ((uint16_t)b[11] << 8)) & 0x07FF); /* 官方未掩码;此处防御性加固 */
    RC_Data.mouse.press_l  = b[12];
    RC_Data.mouse.press_r  = b[13];
    RC_Data.key_v          = b[14];
    RC_Data.key_h          = b[15];

    /* ---- 左拨轮：11bit。官方 sbus_to_rc 未对 wheel 掩码；
     * 此处 &0x07FF 为防御性加固——切掉 b[17] 高 5 位未定义脏位，
     * 合法帧恒等、异常帧不会把脏位当有效值 ---- */
    RC_Data.wheel = (uint16_t)((b[16] | ((uint16_t)b[17] << 8)) & 0x07FF);

    RC_Data.last_tick = HAL_GetTick();
    RC_Data.online = 1;
}

uint8_t RC_IsOnline(uint32_t now_ms)
{
    if ((now_ms - RC_Data.last_tick) > RC_OFFLINE_MS)
        RC_Data.online = 0;
    return RC_Data.online;
}

/* 摇杆原始值 → -1.0 ~ +1.0（带死区） */
float RC_Norm(uint16_t ch)
{
    int32_t v = (int32_t)ch - RC_CH_MID;

    if ((v > -RC_DEADBAND) && (v < RC_DEADBAND))
        return 0.0f;

    v = (v > 0) ? (v - RC_DEADBAND) : (v + RC_DEADBAND);

    float n = (float)v / (float)(RC_CH_MAX - RC_CH_MID);

    if (n >  1.0f) n =  1.0f;
    if (n < -1.0f) n = -1.0f;
    return n;
}

/* ------------------------------------------------------------------ */
/* 控制任务：1kHz 调用一次                                              */
/*   摇杆 → 归一化 → 麦轮混控(含斜坡) → 限幅 → CAN 电流指令              */
/* ------------------------------------------------------------------ */
void RC_Control_Task(void)
{
    uint32_t now = HAL_GetTick();

    /* 1) 通信在线判断：>100ms 没收到新帧 = 遥控器/接收机失联 */
    uint8_t online = RC_IsOnline(now);

#if (RC_ENABLE_SW_POS != 0)
  #if (RC_ENABLE_USE_LEFT)
    uint8_t armed = (RC_Data.s_left  == RC_ENABLE_SW_POS);
  #else
    uint8_t armed = (RC_Data.s_right == RC_ENABLE_SW_POS);
  #endif
#else
    uint8_t armed = 1;
#endif

    uint8_t enabled = (online && armed);

    /* 使能条件必须“连续保持”才放行：失联恢复或拨杆刚到位时不立刻上电，
     * 避免遥控信号抖动导致电机抽搐（软启动门） */
    if (enabled)
    {
        if (rc_enable_since == 0)
        {
            rc_enable_since = now;
        }
        else if ((now - rc_enable_since) < RC_ENABLE_DELAY_MS)
        {
            enabled = 0;                /* 门时间内先不给力，但斜坡已开始回零 */
        }
    }
    else
    {
        rc_enable_since = 0;
    }

    /* 2) 摇杆归一化：死区内为 0，范围 -1.0 ~ +1.0 */
    float fwd  = online ?  RC_Norm(RC_CH_FWD)  : 0.0f;
    float spin = online ? RC_Norm(RC_CH_TURN) : 0.0f;

#if (MECANUM_WHEEL_NUM == 4)
    /* 4 轮补齐后：右摇杆左右 = 横移（平移不转向），自转另给通道再用 spin */
    float strafe = online ? RC_Norm(RC_CH_STRAFE) : 0.0f;
#else
    float strafe = 0.0f;      /* 只有 2 个轮子时无法横移 */
#endif

    /* 3) 麦轮混控：差速/麦轮公式 + 归一化斜坡限速（在 mecanum.c 里） */
    Mecanum_Mix(fwd, strafe, spin, enabled, RC_WheelOut);

    /* 4) 失联超过 RC_FAILSAFE_MS 后强制刹停：斜坡已被旁路，必须硬置零 */
    if (!online && ((now - RC_Data.last_tick) > RC_FAILSAFE_MS))
    {
        for (uint8_t i = 0; i < MECANUM_WHEEL_NUM; i++)
        {
            RC_WheelOut[i] = 0.0f;
        }
        Mecanum_Reset();
    }

    /* 5) 归一化目标 → 安培 → 原始值 → 0x200 帧
     *    CAN2 的 4 个槽位 = 底盘四轮：ID1=左前 ID2=右前 ID3=右后 ID4=左后
     *    （与 MECANUM_DIR[] 的下标顺序、mecanum.h 的 MEC_WHEEL_LF/LB/RB/RF 一一对应） */
    static const float s_wheel_sign[MECANUM_WHEEL_NUM] =
    {
#if (MECANUM_WHEEL_NUM == 4)
        M1_SIGN, M2_SIGN, M3_SIGN, M4_SIGN
#else
        M1_SIGN, M4_SIGN
#endif
    };

    float   max_a = (float)RC_MAX_CURRENT_A;
    int16_t raw[MECANUM_WHEEL_NUM];

    for (uint8_t i = 0; i < MECANUM_WHEEL_NUM; i++)
    {
        float a = s_wheel_sign[i] * RC_WheelOut[i] * max_a;

        /* 硬性绝对限幅：不论参数怎么配，都不允许超过 RC_ABS_CURRENT_A */
        if (a >  RC_ABS_CURRENT_A) a =  RC_ABS_CURRENT_A;
        if (a < -RC_ABS_CURRENT_A) a = -RC_ABS_CURRENT_A;

        /* 安培 → C620 原始值(±16384 ↔ ±20A, 见C620手册) */
        raw[i] = (int16_t)(a / 20.0f * 16384.0f);
    }

    /* 6) 云台 GM6020（ID1）：限位 + 超速兜底 + 斜坡 + 安全开关 */
    float gim  = online ? RC_Norm(RC_CH_GIMBAL) : 0.0f;
    float volt = GM6020_SIGN * gim * GM6020_VOLT_MAX;

    /* ① 反馈超时 → 断电（没有反馈就等于没有限位） */
    if ((HAL_GetTick() - gim6020.last_tick) > GM_FEEDBACK_TIMEOUT_MS)
        volt = 0.0f;

    /* ② 超速兜底：不管角度算得对不对，转太快就断 */
    if (gim6020.rpm > GM_RPM_LIMIT || gim6020.rpm < -GM_RPM_LIMIT)
        volt = 0.0f;


    /* ④ 斜坡：每拍最多变 GM_VOLT_SLEW，避免扭矩冲击 */
    static float volt_prev = 0.0f;
    if (volt > volt_prev + GM_VOLT_SLEW) volt = volt_prev + GM_VOLT_SLEW;
    if (volt < volt_prev - GM_VOLT_SLEW) volt = volt_prev - GM_VOLT_SLEW;

    /* ⑤ 安全开关：=1 时强制零输出（只用来量角度，电机不动） */
 /* ⑦ 云台 pitch：右摇杆垂直 → 6020 ID2（有机械限位） */
    float pit    = online ? RC_Norm(RC_CH_GIMBAL_PITCH) : 0.0f;
    float volt_p = GM_PITCH_SIGN * pit * GM_PITCH_VOLT_MAX;
    int32_t ap   = (int32_t)gim_pitch.angle;

    /* ① 反馈超时 → 断电 */
    if ((HAL_GetTick() - gim_pitch.last_tick) > GM_FEEDBACK_TIMEOUT_MS)
        volt_p = 0.0f;

    /* ② 超速兜底 */
    if (gim_pitch.rpm > GM_RPM_LIMIT || gim_pitch.rpm < -GM_RPM_LIMIT)
        volt_p = 0.0f;

    /* ③ 角度限位：界外只许往回收，界内按比例减速 */
    if (ap >= GM_PITCH_ANGLE_MAX || ap <= GM_PITCH_ANGLE_MIN)
    {
        if ((ap >= GM_PITCH_ANGLE_MAX && volt_p > 0.0f) ||
            (ap <= GM_PITCH_ANGLE_MIN && volt_p < 0.0f))
            volt_p = 0.0f;
    }
    else if (ap > GM_PITCH_ANGLE_MAX - GM_PITCH_SOFT_ZONE)
    {
        volt_p *= (float)(GM_PITCH_ANGLE_MAX - ap) / (float)GM_PITCH_SOFT_ZONE;
    }
    else if (ap < GM_PITCH_ANGLE_MIN + GM_PITCH_SOFT_ZONE)
    {
        volt_p *= (float)(ap - GM_PITCH_ANGLE_MIN) / (float)GM_PITCH_SOFT_ZONE;
    }

    /* ④ 斜坡 */
    static float vp_prev = 0.0f;
    if (volt_p > vp_prev + GM_VOLT_SLEW) volt_p = vp_prev + GM_VOLT_SLEW;
    if (volt_p < vp_prev - GM_VOLT_SLEW) volt_p = vp_prev - GM_VOLT_SLEW;
    vp_prev = volt_p;

    /* ⑧ 摩擦轮：左拨杆上 → 3508 ID2/ID3 反向同电流 */
#if (FRIC_ENABLE_USE_LEFT)
    uint8_t fric_sw = (RC_Data.s_left  == FRIC_ENABLE_SW);
#else
    uint8_t fric_sw = (RC_Data.s_right == FRIC_ENABLE_SW);
#endif

    float fric_tgt = (online && fric_sw) ? FRIC_CURRENT_A : 0.0f;

    if (dj_motor[1].temp > FRIC_TEMP_LIMIT || dj_motor[2].temp > FRIC_TEMP_LIMIT)
        fric_tgt = 0.0f;
    if ((HAL_GetTick() - dj_motor[1].last_tick) > FRIC_TIMEOUT_MS ||
        (HAL_GetTick() - dj_motor[2].last_tick) > FRIC_TIMEOUT_MS)
        fric_tgt = 0.0f;

    static float fric_a = 0.0f;
    if      (fric_tgt > fric_a + FRIC_RAMP_A_PER_MS) fric_a += FRIC_RAMP_A_PER_MS;
    else if (fric_tgt < fric_a - FRIC_RAMP_A_PER_MS) fric_a -= FRIC_RAMP_A_PER_MS;
    else                                             fric_a  = fric_tgt;

    int16_t fric_l;
    int16_t fric_r;

    if (fric_tgt > 0.0001f)
    {
        /* 正常出力：两轮反向，把弹丸往外挤 */
        fric_l = (int16_t)(FRIC_L_SIGN * fric_a / 20.0f * 16384.0f);
        fric_r = (int16_t)(FRIC_R_SIGN * fric_a / 20.0f * 16384.0f);
    }
    else
    {
        /* 松杆：反向小电流主动刹车，否则纯靠惯性会滑很久 */
        int16_t brake = (int16_t)(FRIC_BRAKE_A / 20.0f * 16384.0f);

        if (dj_motor[1].rpm >  FRIC_BRAKE_RPM)
        {
            fric_l = -brake;
        }
        else if (dj_motor[1].rpm < -FRIC_BRAKE_RPM)
        {
            fric_l =  brake;
        }
        else
        {
            fric_l = 0;
        }

        if (dj_motor[2].rpm >  FRIC_BRAKE_RPM)
        {
            fric_r = -brake;
        }
        else if (dj_motor[2].rpm < -FRIC_BRAKE_RPM)
        {
            fric_r =  brake;
        }
        else
        {
            fric_r = 0;
        }
    }

    /* ⑤ 安全开关：yaw / pitch / 摩擦轮 全部清零 */
#if GM_SAFE_NO_OUTPUT
    volt   = 0.0f;
    volt_p = 0.0f;
    fric_l = 0;
    fric_r = 0;
#endif
    volt_prev = volt;           /* ★ 记下本拍输出，下一拍才能继续爬 */

    /* ⑥ 发送：CAN2 = 底盘四轮 + yaw；CAN1 = 摩擦轮 + pitch */
#if (FRIC_USE_CAN1)
  #if (MECANUM_WHEEL_NUM == 4)
    DJ3508_Send_I_Can2(raw[0], raw[1], raw[2], raw[3]); /* CAN2：四轮 0x200 槽位全用 */
  #else
    DJ3508_Send_I_Can2(raw[0], 0, 0, raw[1]);           /* CAN2：ID1 底盘左 / ID4 底盘右 */
  #endif
    DJ6020_Send_V_Can2((int16_t)volt, 0);       /* CAN2：yaw ID1 */
    DJ3508_Send_Can1(fric_l, fric_r);           /* CAN1：摩擦轮 ID2 / ID3 */
    DJ6020_Send_Can1((int16_t)volt_p);          /* CAN1：pitch ID2 */
#else
  #if (MECANUM_WHEEL_NUM != 2)
    #error "four-wheel chassis requires FRIC_USE_CAN1=1 (ID2/ID3 slot conflict)"
  #endif
    DJ3508_Send_I_Can2(raw[0], fric_l, fric_r, raw[1]);
    DJ6020_Send_V_Can2((int16_t)volt, (int16_t)volt_p);
#endif
}

/* ------------------------------------------------------------------ */
/* UART 空闲中断回调：一帧收完由 HAL 调用                                */
/* ------------------------------------------------------------------ */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
        /* 只有真正打开了 D-Cache 才需要做 cache 维护。
         * 本工程 D-Cache 关闭（SCB->CCR.DC=0），实测此处若无条件调用
         * SCB_InvalidateDCache_by_Addr() 会在第一帧到达时触发
         * HardFault（CFSR.IMPRECISERR），导致摇杆永远没反应。 */
        if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)dbus_buf, sizeof(dbus_buf));
        }

        if (Size >= DBUS_FRAME_LEN)
        {
            /* 收到多帧时取最后一帧完整数据 */
            RC_Decode(&dbus_buf[Size - DBUS_FRAME_LEN]);
        }

        RC_StartReceive();   /* 普通模式，需要重新启动 */
    }
}

/* ------------------------------------------------------------------ */
/* UART 错误回调（ORE/FE/NE/PE）                                        */
/*   HAL 一旦报错就会把接收整个拆掉（关 IDLE/DMA），如果这里不重开，      */
/*   接收链会永久静默 —— 这正是“摇杆怎么推都没反应”的典型死法之一。      */
/* ------------------------------------------------------------------ */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        RC_StartReceive();
    }
}























