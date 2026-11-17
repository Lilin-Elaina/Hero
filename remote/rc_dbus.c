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
#include "ins.h"
#include <string.h>
#include <math.h>

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
{//启动dbus数据的idle dma接收中断
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

/*清除摇杆数据，在线状态，确定拨杆，开启接收*/
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
/* ==================================================================
 *  云台 pitch 双环（6020 ID2，1 kHz 调用）
 *
 *  三个状态：0=未接管  1=回中  2=正常
 *  未接管时要求摇杆居中，且参考角永远从"当前位置"起步 → 不会瞬移、
 *  不会有劣弧、不会一拨使能就跟着摇杆飞。
 *
 *  采用位置环 + 速度环的双环控制，位置环输出目标速度，速度环输出电压。
 * ================================================================== */


 /*求绝对值 */
static float pitch_abs(float v)
 {
     return (v < 0.0f) ? -v : v;
 }

/*对值进行范围限定*/
static float pitch_clamp(float v, float lo, float hi)
{
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return v;
}

/* 把 cur 朝 tgt 限速推进一步（counts） */
static float pitch_step(float cur, float tgt, float max_step)
{
    float d = tgt - cur;

    if (d >  max_step) d =  max_step;
    if (d < -max_step) d = -max_step;
    return cur + d;
}
/*通过位置与速度环对pitch电机进行控制*/
static float RC_Pitch_Control(uint8_t online, uint8_t armed, float stick)
{
    static float s_ref = 0.0f;          /* 参考角，连续量（counts） */
    static float s_i   = 0.0f;          /* 速度环积分（电压单位）   */
    static uint8_t  s_stage = 0;        /*状态*/
    static uint8_t   s_homed   = 0;     /*是否回中*/
    static uint32_t  s_home_since = 0;  /*回中时间判定基准*/
    static float     s_stick_f = 0.0f;  /*滤波后的摇杆值*/
    static float     s_v = 0.0f;        /*手控参考速度*/


    const float lo = (float)(GM_PITCH_ANGLE_MIN + GM_PITCH_ANGLE_MARGIN);
    const float hi = (float)(GM_PITCH_ANGLE_MAX - GM_PITCH_ANGLE_MARGIN);
    const float home = (float)GM_PITCH_HOME_ANGLE;

 if (!GM_PITCH_CALIBRATED || !online || !armed)
    {
        s_stage = 0;
        s_i     = 0.0f;
    
        return 0.0f;
    }

    float cur = (float)gim_pitch.angle;
    float rpm = GM_PITCH_RPM_SIGN * (float)gim_pitch.rpm;
    float err, rpm_des, volt;


 if ((HAL_GetTick() - gim_pitch.last_tick) > GM_FEEDBACK_TIMEOUT_MS ||
        gim_pitch.temp > GM_PITCH_TEMP_LIMIT ||
        pitch_abs(rpm) > GM_RPM_LIMIT ||
        cur < (float)GM_PITCH_ANGLE_MIN ||
        cur > (float)GM_PITCH_ANGLE_MAX)
    {
        s_stage = 0;
        s_i     = 0.0f;
        return 0.0f;
    }

     s_stick_f += (stick - s_stick_f) * 0.2f;

    /* ③ 接管条件：刚开始上电只看摇杆是否居中 */
    if (s_stage == 0 && pitch_abs(stick) > GM_PITCH_STICK_DEAD)
    {
        return 0.0f;
    }
    /* 本拍参考角的起点，供下面的参考速度前馈使用 */
    const float s_ref_old = s_ref;

  /*此时遥控器在线，开始进行判断*/
if(s_stage == 0)
{
    s_ref = cur;
    s_i   = 0.0f;
    s_v   = 0.0f;
    s_home_since = HAL_GetTick() | 1u;     /* 接管时记一次起点，供回中计时用 */
    s_stage = ( GM_PITCH_HOME_ENABLE && !s_homed) ? 1 : 2;
    return 0.0f;
}

/*在线之后进行判断，若未回中，开始回中*/
if(s_stage == 1)
{
    s_v = 0.0f;
    /*若一直没符合下面置2的三个条件时触发*/
    if ((HAL_GetTick() - s_home_since) > GM_PITCH_HOME_TIMEOUT_MS)
    {
        /* 回中失败：就地按住，不反复顶、也不锁死，可控 */
        s_ref        = cur;
        s_i          = 0.0f;
        s_homed      = 1;
        s_stage      = 2;
        s_home_since = 0;
        return 0.0f;
    }
      s_ref = pitch_step(s_ref, home,GM_PITCH_HOME_RATE * 0.001f );

      if (pitch_abs(s_ref - home) <= 1.0f &&
          pitch_abs(rpm)        <  GM_PITCH_CATCH_RPM &&
          pitch_abs(cur - home) <= 60.0f)
    {
       s_ref  = home;
       s_homed = 1;
       s_stage = 2;
       s_home_since = 0;
    }
}
else
{
      float v_tgt = s_stick_f * GM_PITCH_MANUAL_RATE;
      float braking = (v_tgt * s_v < 0.0f)  ||
      (pitch_abs(v_tgt) < pitch_abs(s_v)) ;

      float a = (braking) ? GM_PITCH_DECEL : GM_PITCH_ACCEL;
      float dv = a * 0.001f;

      if(v_tgt > s_v + dv)      s_v += dv;
      else if(v_tgt < s_v - dv) s_v -= dv;
      else                      s_v = v_tgt;


      s_ref += s_v * 0.001f;


}

if(s_ref < lo) s_ref = lo;
if(s_ref > hi) s_ref = hi;

    /* 参考速度前馈：参考角这一拍走了多少 → 折算成 rpm，直接叠加到 rpm_des。
     * 位置环只有 P，匀速跟踪时必然留下恒定滞后 err = RATE/(136.53*POS_KP)，
     * 这个非零误差会被速度环积分一路累积到 I_MAX；参考一停、积分却还满着，
     * 表现出来就是"冲过头 + 回调很慢"。加了前馈，跟踪误差归零，
     * 积分只负责扛重力那一点。
     * 136.53 = 8192/60，即 1 rpm 对应的 counts/s。 */
    {
        float v_ff = (s_ref - s_ref_old) * 1000.0f / 136.53f;   /* counts/s → rpm */

        err     = s_ref - cur;
        rpm_des = pitch_clamp(GM_PITCH_POS_KP * err + v_ff,
                              -GM_PITCH_RPM_MAX, GM_PITCH_RPM_MAX);
    }


err = rpm_des - rpm;
volt = GM_PITCH_SPD_KP * err + s_i + GM_PITCH_GRAV_FF;

if(pitch_abs(volt) < GM_PITCH_VOLT_MAX || (volt >= GM_PITCH_VOLT_MAX && err < 0.0f) || (volt <= -GM_PITCH_VOLT_MAX && err > 0.0f))
{
s_i += GM_PITCH_SPD_KI * err * 0.001f;
s_i = pitch_clamp(s_i, -GM_PITCH_I_MAX,GM_PITCH_I_MAX);
}

volt = pitch_clamp(GM_PITCH_SPD_KP* err + s_i + GM_PITCH_GRAV_FF, -GM_PITCH_VOLT_MAX,GM_PITCH_VOLT_MAX);

    /* 软墙：距离边界 soft 以内，把"往外推"的力按比例衰减到 0，
       避免"出力/不出力"每毫秒硬切换导致的高频起振 */
    const float soft = 120.0f;

    if (cur <= lo)
    {
        /* 已经在墙外：只允许往回走的力 */
        if (volt < 0.0f) volt = 0.0f;
    }
    else if (cur >= hi)
    {
        if (volt > 0.0f) volt = 0.0f;
    }
    else if (cur < lo + soft && volt < 0.0f)
    {
        /* 靠近下限：往下推的力按比例减小，贴墙时正好为 0 */
        volt *= (cur - lo) / soft;
    }
    else if (cur > hi - soft && volt > 0.0f)
    {
        /* 靠近上限：往上推的力按比例减小 */
        volt *= (hi - cur) / soft;
    }

return GM_PITCH_SIGN * volt;

}
/*=============底盘四轮速度闭环===========*/
 volatile uint8_t RC_WheelFault = 0; /*轮子故障标志位，1表示有轮子故障，0表示正常*/

static float s_wheel_i[MECANUM_WHEEL_NUM];

/* 目标轮速镜像（只给调试器看，控制里不读） */
float RC_WheelRef[4] = {0.0f};
/* 调试镜像：yaw 闭环内部量（只读，给 Ozone 看） */
volatile float RC_YawRef    = 0.0f;   /* 目标角 counts */
volatile float RC_YawErr    = 0.0f;   /* 角度误差 counts */
volatile float RC_YawRpmDes = 0.0f;   /* 位置环输出的目标转速 rpm */
volatile float RC_YawVolt   = 0.0f;   /* 斜坡之前的电压指令 */
volatile float RC_YawVoltOut= 0.0f;   /* 斜坡之后、真正发出去的电压 */
volatile float RC_YawI      = 0.0f;   /* 速度环积分 */
volatile float RC_HoldWz    = 0.0f;   /* 稳向看到的枪口世界角速度 deg/s */
volatile float RC_HoldRate  = 0.0f;   /* 稳向折算出的补偿角速度 deg/s */

static float wheel_abs(float x )
{
    return (x < 0.0f) ? -x : x;
}

static float wheel_clamp(float v, float lo, float hi)
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

  /*单轮速度  PI  :前馈 + P +条件积分 + 输出限幅*/
static float RC_Wheel_Speed_PI(uint8_t id, float ref , float rpm, float limit)
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

/* ======================= 云台 yaw 角度闭环 ======================= */
static float   s_yaw_ref   = 0.0f;   /* 目标角（counts） */
static float   s_yaw_i     = 0.0f;   /* 速度环积分（电压单位） */
static uint8_t s_yaw_ready = 0u;
static float   s_psi_deg   = 0.0f;   /* 枪口世界航向角（度，陀螺积分） */
static float   s_psi_ref   = 0.0f;   /* 航向锁存基准（度） */
static float   s_hold_i    = 0.0f;   /* 稳向积分（deg/s），只在底盘带着转时累加 */

/* 小陀螺专用环的状态（与上面互不共享，靠 RC_Yaw_Handover 交接） */
static float   s_spin_ref   = 0.0f;  /* 世界航向目标 deg（多圈连续） */
static float   s_spin_i     = 0.0f;  /* 速度环积分（电压单位） */
static uint8_t s_spin_ready = 0u;
static uint16_t s_spin_bad  = 0u;   /* 失控计数（WorldYawRate 长时间超限） */
static uint8_t  s_yaw_slew_kick = 0u;  /* 交接后把输出斜坡清零的标志 */

volatile float RC_SpinYaw = 0.0f;
volatile float RC_SpinWch = 0.0f;
volatile float RC_SpinErr = 0.0f;
volatile float RC_YawSpinOut = 0.0f;   /* 小陀螺环上一拍真正发出去的电压（斜坡+限幅之后），供积分回算 */

static float yaw_wrap(float d)
{
    while (d >  4096.0f) d -= 8192.0f;
    while (d < -4096.0f) d += 8192.0f;
    return d;
}

/* 枪口相对车头的夹角 α（度，-180 ~ +180）
 * 逆时针（俯视）为正，即 α>0 表示枪口偏向车头左侧 */
volatile float RC_GunAngle = 0.0f;

float RC_GetOffsetAngle(void)
{
    float d = yaw_wrap((float)gim6020.angle - GM_YAW_ALIGN_COUNT);
    d = d * 360.0f / 8192.0f;          /* counts → 度 */
    RC_GunAngle = d;
    return d;
}

float RC_Yaw_Control(uint8_t online, float stick)
{
    const float cur = (float)gim6020.angle;
    /* 反馈超时等同离线：目标/积分/接管状态一起清，恢复后按当时的实际角重新接管
     * （否则会拿最后一帧的旧角度当目标，恢复瞬间被拽一下） */
    const uint8_t fresh = ((HAL_GetTick() - gim6020.last_tick) <= GM_FEEDBACK_TIMEOUT_MS) ? 1u : 0u;
    float d_ref, err, rpm_d, err_v, volt;
    float hold_rate = 0.0f;      /* 稳向折算的关节角速度（deg/s），0=未介入 */

    if ((online == 0u) || (fresh == 0u) || (s_yaw_ready == 0u)) {
        s_yaw_ref   = cur;
        s_yaw_i     = 0.0f;
        s_yaw_ready = (uint8_t)((online != 0u) && (fresh != 0u));
        s_psi_deg   = 0.0f;          /* 航向是相对量，离线时整段作废，恢复后重新锁 */
        s_psi_ref   = 0.0f;
        s_hold_i    = 0.0f;
        RC_YawRef    = s_yaw_ref;
        RC_YawErr    = 0.0f;
        RC_YawRpmDes = 0.0f;
        RC_YawVolt   = 0.0f;
        RC_YawI      = s_yaw_i;
        return 0.0f;
    }

    /* ① 摇杆 → 目标角增量（松杆保持），沿用原开环手感 */
    d_ref      = GM6020_SIGN * GM_YAW_STICK_RATE * stick * 0.001f;
    s_yaw_ref += d_ref;

    /* ①.2 松杆就不攒积分
     * 云台 yaw 是水平轴，没有重力要顶；松杆后位置环的 P 项足够把角度按住。
     * 积分只在"主动推杆"时有用，松杆还留着它，就会慢慢把枪口顶偏再弹回来。 */
    if (d_ref == 0.0f) {
        s_yaw_i = 0.0f;
    }

    /* ①.5 世界稳向 —— 2026-11-13 第四次修订：速度 P + 积分
     * 只有比例时残差 = ω/(1+K)：K=2 也还剩 33%，转 5 s 就丢掉 1/3 的角度，
     * "保持世界朝向"永远做不干净。这里在**枪口角速度**上串一个积分
     * （对象是 ω 不是角度，不构成第二个积分器，不会像上次那样自激），
     * 稳态把残差压到 0；条件积分 + 限幅，防止长时间旋转把积分灌满。 */
    {
        float wz_ms  = GM_YAW_HOLD_SIGN * -(float)INS_Gyro[0] * 57.2957795f;  /* 枪口世界角速度 deg/s */
        float wz_cmd = d_ref * 1000.0f / GM_YAW_COUNT_PER_DEG;               /* 操作手要的关节角速度 deg/s */
        /* ★误差取"实测 − 指令"，而不是"实测 − 0"，也不再靠 d_ref==0 开门：
         *   遥控器零点只要偏 3 个 count，d_ref 就不为 0，原来的门控会让稳向**整段失效**，
         *   表现就是"稳一下就被带走"。现在给杆时 wz_err 自动扣掉操作手那部分，
         *   稳向只补"底盘带出来的"转速，推杆不会变粘。 */
        float wz_err = wz_ms - wz_cmd;

        if ((wz_err > GM_YAW_HOLD_DEAD) || (wz_err < -GM_YAW_HOLD_DEAD)) {
            float rate = -(GM_YAW_HOLD_K * wz_err + s_hold_i);   /* deg/s */

            if (rate > GM_YAW_HOLD_RATE_MAX) {
                rate = GM_YAW_HOLD_RATE_MAX;
            } else if (rate < -GM_YAW_HOLD_RATE_MAX) {
                rate = -GM_YAW_HOLD_RATE_MAX;
            } else {
                s_hold_i += GM_YAW_HOLD_KI * wz_err * 0.001f;    /* 只在未饱和时积分 */
                if (s_hold_i >  GM_YAW_HOLD_RATE_MAX) s_hold_i =  GM_YAW_HOLD_RATE_MAX;
                if (s_hold_i < -GM_YAW_HOLD_RATE_MAX) s_hold_i = -GM_YAW_HOLD_RATE_MAX;
            }

            s_yaw_ref += rate * GM_YAW_COUNT_PER_DEG * 0.001f;
            hold_rate  = rate;     /* ★同时作为速度前馈：只挪目标角、不加转速前馈的话，
                                    *   位置环要靠"误差"才生得出转速，稳态滞后 = 转速/KP，
                                    *   转起来就跟不上目标 → 枪口还是被底盘拖着走 */
            RC_HoldWz   = wz_ms;
            RC_HoldRate = rate;
        } else {
            s_hold_i = 0.0f;   /* 不转就清积分，免得残留偏置让关节自己慢慢爬 */
        }
    }

    s_yaw_ref  = cur + yaw_wrap(s_yaw_ref - cur);   /* 折到 ±4096，防绕圈 */

    /* ② 位置 P + 速度前馈 */
    err   = yaw_wrap(s_yaw_ref - cur);

    /* ②.5 到位清零积分
     * 现象：快转到位、定住 1~2 s 后又小幅度回摆一下。
     * 原因：到位瞬间速度环还留着正向积分，位置环慢慢把角度拽回来，
     *       积分一点点卸，卸完才不动 —— 那一下回摆就是它在卸。
     * 做法：位置误差和转速都小到"已经到位"时，直接把积分清掉。 */
    if ((err < GM_YAW_ARRIVE_CNT) && (err > -GM_YAW_ARRIVE_CNT) &&
        (gim6020.rpm < GM_YAW_ARRIVE_RPM) && (gim6020.rpm > -GM_YAW_ARRIVE_RPM)) {
        s_yaw_i = 0.0f;
    }

    {
        /* D 项单独限幅：不封顶的话 250 rpm × 0.6 = 150 rpm，占满 RPM_MAX 的 60%，
         * 减速段 D 项一直满舵，冲过目标再被拉回 —— 就是"来回拟合"的根源 */
        float d_term = -GM_YAW_POS_DIR * GM_YAW_ANG_KD * (float)gim6020.rpm;

        if (d_term >  GM_YAW_ANG_D_MAX) d_term =  GM_YAW_ANG_D_MAX;
        if (d_term < -GM_YAW_ANG_D_MAX) d_term = -GM_YAW_ANG_D_MAX;

        rpm_d = GM_YAW_POS_DIR * GM_YAW_ANG_KP * err
              + d_ref * 1000.0f / 136.53f
              + hold_rate * (1.0f / 6.0f)   /* deg/s → rpm：稳向的速度前馈 */
              + d_term;
    }
    if (rpm_d >  GM_YAW_RPM_MAX) rpm_d =  GM_YAW_RPM_MAX;
    if (rpm_d < -GM_YAW_RPM_MAX) rpm_d = -GM_YAW_RPM_MAX;

    /* ③ 速度 PI（条件积分 + 限幅） */
    err_v = rpm_d - (float)gim6020.rpm;
    volt  = GM_YAW_SPD_KP * err_v + s_yaw_i;

    if ((volt < GM_YAW_VOLT_MAX) && (volt > -GM_YAW_VOLT_MAX)) {
        s_yaw_i += GM_YAW_SPD_KI * err_v * 0.001f;
        if (s_yaw_i >  GM_YAW_I_MAX) s_yaw_i =  GM_YAW_I_MAX;
        if (s_yaw_i < -GM_YAW_I_MAX) s_yaw_i = -GM_YAW_I_MAX;
    }
    volt = GM_YAW_SPD_KP * err_v + s_yaw_i;

    if (volt >  GM_YAW_VOLT_MAX) volt =  GM_YAW_VOLT_MAX;
    if (volt < -GM_YAW_VOLT_MAX) volt = -GM_YAW_VOLT_MAX;

    /* 调试镜像 */
    RC_YawRef    = s_yaw_ref;
    RC_YawErr    = err;
    RC_YawRpmDes = rpm_d;
    RC_YawVolt   = volt;
    RC_YawI      = s_yaw_i;

    return volt;
}

/* ======================= 云台 yaw：小陀螺专用（世界航向闭环） =======================
 * 为什么单独一套：小陀螺下"保持世界朝向"是个跟踪问题——底盘在匀速转，
 * 枪口必须**持续**反向转。用位置环慢慢攒误差的方式（RC_Yaw_Control 里的稳向）
 * 带宽不够，稳态滞后 = 转速/KP，转起来就被拖走。
 * 这里直接把关系摆平：ω_world = ω_chassis + ω_joint，要 ω_world = w_cmd
 * → ω_joint = w_cmd − ω_chassis；ω_chassis 用实测（世界角速度 − 关节角速度），
 * 所以底盘转多快当拍就知道，不用等误差。
 */
float RC_Yaw_Spin_Control(uint8_t online, float stick)
{
    const float cur = (float)gim6020.angle;
    const float alpha_deg = RC_GetOffsetAngle();   /* 枪口相对车头，度（关节角） */
    const uint8_t fresh = ((HAL_GetTick() - gim6020.last_tick) <= GM_FEEDBACK_TIMEOUT_MS) ? 1u : 0u;
    float aim, e, w_cmd, w_chassis, rpm_d, err_v, volt;

    if ((online == 0u) || (fresh == 0u) || (s_spin_ready == 0u) || (INS_AhrsReady == 0u)) {
        s_spin_ref   = (float)INS_WorldYaw;
        s_spin_i     = 0.0f;
        s_spin_ready = (uint8_t)((online != 0u) && (fresh != 0u) && (INS_AhrsReady != 0u));
        RC_SpinYaw   = (float)INS_WorldYaw;
        RC_SpinWch   = 0.0f;
        RC_SpinErr   = 0.0f;
        RC_YawRef    = cur;
        RC_YawErr    = 0.0f;
        RC_YawRpmDes = 0.0f;
        RC_YawVolt   = 0.0f;
        RC_YawI      = s_spin_i;
        return 0.0f;
    }

    /* ① 摇杆 → 世界目标角速度；松杆后目标保持（不回中、不回弹） */
    aim = GM6020_SIGN * SPIN_YAW_AIM_RATE * stick;      /* deg/s */
    s_spin_ref += aim * 0.001f;

    /* ② 世界航向误差，折到 ±180（INS_WorldYaw 是连续量） */
    e = s_spin_ref - (float)INS_WorldYaw;
    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;

    /* ③ 实测底盘角速度 = 枪口世界角速度 − 关节角速度（1 rpm = 6 °/s） */
    w_chassis = (float)INS_WorldYawRate - 6.0f * (float)gim6020.rpm;

    /* ③.5 失控兜底：世界角速度长时间超 ±800 °/s，说明这条环在正反馈
     * （符号/方向接反），收油并把目标拉回当前航向，绝不让它一直疯转。
     * 正常工况下 ω_world 被钉在 0 附近，不可能触发。 */
    if ((INS_WorldYawRate > 800.0f) || (INS_WorldYawRate < -800.0f)) {
        if (s_spin_bad < 300u) s_spin_bad++;
        if (s_spin_bad >= 300u) {
            s_spin_ref = (float)INS_WorldYaw;
            s_spin_i   = 0.0f;
            s_spin_bad = 0u;
            RC_SpinWch = w_chassis;
            RC_YawVolt = 0.0f;
            return 0.0f;
        }
    } else {
        s_spin_bad = 0u;
    }

#if (SPIN_YAW_USE_JOINT_POS)
    /* ④ 世界目标 → 关节目标：ψ_c = ψ枪口 − α，α_ref = ψ_ref − ψ_c
     * 外层直接作用在**关节角**上：手推枪口时位置环立刻出回复力矩（有刚度），
     * 而不是速率环那种"先被推走、再按 POS_KP 慢慢追"的软响应。
     * 底盘转速前馈 −ω_c/6 保证边转边跟得住。 */
    {
        float psi_c = (float)INS_WorldYaw - alpha_deg;   /* 车体世界航向 */
        float a_err = (s_spin_ref - psi_c) - alpha_deg;  /* 关节角误差（deg） */

        rpm_d = GM_YAW_ANG_KP * (a_err * 22.7556f) - w_chassis * (1.0f / 6.0f);
        w_cmd = rpm_d * 6.0f;
    }
#else
    /* ④ 目标世界角速度 = 摇杆那份 + 把误差拉回 */
    w_cmd = aim + SPIN_YAW_POS_KP * e;

    if (w_cmd >  SPIN_YAW_RATE_MAX) w_cmd =  SPIN_YAW_RATE_MAX;
    if (w_cmd < -SPIN_YAW_RATE_MAX) w_cmd = -SPIN_YAW_RATE_MAX;

    /* ⑤ 关节该转多少：ω_joint = w_cmd − ω_chassis，换成 rpm */
    rpm_d = (w_cmd - w_chassis) * (1.0f / 6.0f);
#endif

    if (rpm_d >  SPIN_YAW_RPM_MAX) rpm_d =  SPIN_YAW_RPM_MAX;
    if (rpm_d < -SPIN_YAW_RPM_MAX) rpm_d = -SPIN_YAW_RPM_MAX;

    /* ⑥ 关节速度环：反电势前馈 + P + 慢积分。
     * 前馈是这套的关键：6020 想稳在 33 rpm，必须有 ≈3133 单位的反电势电压；
     * 只靠 P 拿，就要 3133/300 ≈ 10 rpm 的稳态速度误差，
     * 折算到航向上就是几十度——这就是"一段时间后被带偏"的来历。 */
    err_v = rpm_d - (float)gim6020.rpm;
    volt  = SPIN_YAW_KV * rpm_d + SPIN_YAW_SPD_KP * err_v + s_spin_i;

    /* 抗饱和：按**上一拍实发**的电压回算，而不是"请求值没超限就积分"。
     * 200/拍的斜坡要 90 ms 才拉到 18000，这期间请求值一直"没超限"，
     * 积分就一路涨；等斜坡追上，积分的钱已经花出去了 → 过冲 → 反打。
     * 回算项必须带 dt（KAW 单位 1/s），否则等于放大 1/dt 倍。 */
    {
        float v_raw = volt;
        s_spin_i += (SPIN_YAW_KI * err_v
                   + SPIN_YAW_KAW * (RC_YawSpinOut - v_raw)) * 0.001f;
        if (s_spin_i >  SPIN_YAW_I_MAX) s_spin_i =  SPIN_YAW_I_MAX;
        if (s_spin_i < -SPIN_YAW_I_MAX) s_spin_i = -SPIN_YAW_I_MAX;
    }
    volt = SPIN_YAW_KV * rpm_d + SPIN_YAW_SPD_KP * err_v + s_spin_i;

    if (volt >  GM_YAW_VOLT_MAX) volt =  GM_YAW_VOLT_MAX;
    if (volt < -GM_YAW_VOLT_MAX) volt = -GM_YAW_VOLT_MAX;

    /* 调试镜像 */
    RC_SpinYaw   = (float)INS_WorldYaw;
    RC_SpinWch   = w_chassis;
    RC_SpinErr   = e;
    RC_YawRef    = s_spin_ref;      /* 注意：小陀螺下这里是"世界目标角 deg"，不是 counts */
    RC_YawErr    = e;
    RC_YawRpmDes = rpm_d;
    RC_YawVolt   = volt;
    RC_YawI      = s_spin_i;

    return volt;
}

/* 模式切换交接：进/出小陀螺时两边状态都清，谁都从"当前实际"重新开始，
 * 免得接手的一方拿着几秒前的旧目标把枪口拽一下。 */
void RC_Yaw_Handover(void)
{
    s_yaw_ref    = (float)gim6020.angle;
    s_yaw_i      = 0.0f;
    s_yaw_ready  = 0u;          /* 下一拍 RC_Yaw_Control 自己按实际角重新接管 */

    s_spin_ref   = (float)INS_WorldYaw;
    s_spin_i     = 0.0f;
    s_spin_ready = 0u;

    /* 输出斜坡也要清：上一模式可能正卡在一个大电压的斜坡上，直接交接等于把
     * 那段扭矩原封不动递给新的环 —— 表现为切模式瞬间"拽一下"。 */
    s_yaw_slew_kick = 1u;
}

/* ------------------------------------------------------------------ */
/* 控制任务：1kHz 调用一次                                              */
/*   摇杆 → 归一化 → 麦轮混控(含斜坡) → 限幅 → CAN 电流指令              */
/* ------------------------------------------------------------------ */
void RC_Control_Task(void)
{     /*通信在线判断*/
    uint32_t now = HAL_GetTick();

    uint8_t online = RC_IsOnline(now);

#if (RC_ENABLE_SW_POS != 0)
  #if (RC_ENABLE_USE_LEFT)
    #define RC_ARM_SW   (RC_Data.s_left)
  #else
    #define RC_ARM_SW   (RC_Data.s_right)
  #endif
#else
  #define RC_ARM_SW     1
#endif

    /* 右拨杆三挡：下=底盘失能 / 中=跟随 / 上=小陀螺
     * 去抖：位置连续 40ms 不变才认（上电慢 40ms），一变化就重新计时；
     * 位域单帧错乱读到的“别的值”一律按失能处理（安全侧）。 */
#if (RC_ENABLE_SW_POS == 0)
    uint8_t armed     = 1u;
    uint8_t follow_on = 0u;
    uint8_t spin_mode = 0u;
#else
    static uint8_t  s_sw_prev = 0u;
    static uint16_t s_sw_cnt  = 0u;

    uint8_t sw_now = (uint8_t)RC_ARM_SW;

    if (sw_now == s_sw_prev) {
        if (s_sw_cnt < 60u) s_sw_cnt++;
    } else {
        s_sw_prev = sw_now;
        s_sw_cnt  = 0u;
    }

    uint8_t sw_ok = (s_sw_cnt >= 40u);

    uint8_t armed     = 0u;
    uint8_t follow_on = 0u;
    uint8_t spin_mode = 0u;

    if (sw_ok) {
        if (sw_now == RC_SW_MID) {          /* 中位：底盘跟随 */
            armed     = 1u;
            follow_on = 1u;
        } else if (sw_now == RC_SW_UP) {    /* 上位：小陀螺，底盘自己转 */
            armed     = 1u;
            spin_mode = 1u;
        }
    }

    /* 没有陀螺就没有稳向：此时跟随会变成"自己一直转"，宁可不跟 */
    if (INS_Ready == 0u) follow_on = 0u;

    /* 小陀螺的世界航向完全依赖 IMU：姿态没收敛/零偏没标定完就撤掉自动自转，
     * 退化为手动（底盘不转、yaw 走手控），绝不偷偷退回旧环继续转 */
    if ((INS_Ready == 0u) || (INS_CalDone == 0u)) spin_mode = 0u;
#endif

    /* 模式切换 → 两个 yaw 控制器交接状态（避免拿旧目标把枪口拽一下） */
    {
        static uint8_t s_spin_prev = 0u;

        if (spin_mode != s_spin_prev) {
            s_spin_prev = spin_mode;
            RC_Yaw_Handover();
        }
    }

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

    /* 小陀螺模式：底盘自己转起来，转轮再叠加一点微调 */
    if (spin_mode != 0u)
    {
        spin = GM_SPIN_BASE + RC_SPIN_TRIM * spin;
        if (spin >  1.0f) spin =  1.0f;
        if (spin < -1.0f) spin = -1.0f;
    }
    float strafe = 0.0f;
#if (MECANUM_WHEEL_NUM == 4)
   
   strafe  =online ? RC_Norm(RC_CH_STRAFE) : 0.0f;

#endif

#if (GM_GUN_ANGLE_ENABLE)
    float alpha_deg = RC_GetOffsetAngle();         /* 枪口相对车头，度，+ 为左 */

    /* 枪口系 → 车体系：摇杆方向按枪口朝向解释
     * 实车标定（2026-11-13）：α=0 / α=±180 正确，α=±90 两组全反
     * → 旋转方向取反（等价于用 -α 旋转），0 和 180 不受影响
     *   fwd_b = cosα·vx_g - sinα·vy_g
     *   vy_b  = sinα·vx_g + cosα·vy_g
     * α=0 时必须是单位阵（推左就往左），这条是判据 */
    {
        float a = GM_GUN_ANGLE_SIGN * alpha_deg * 0.017453292f;
        float c = cosf(a);
        float s = sinf(a);
        float fwd_g    = fwd;
        float strafe_g = strafe;

        fwd    = c * fwd_g - s * strafe_g;
        strafe = s * fwd_g + c * strafe_g;
    }

    /* 底盘跟随（右拨杆中位）：车头追向枪口，α→0
     * 只用云台编码器，零漂、不积分；死区内不动，避免到位后原地左右蹭
     * 2026-11-13：纯 P 在小角度只剩 0.05 的自转指令，推不动车 → 磨蹭；
     * 加一条小积分，死区外慢慢攒，到位立刻清，专门治"最后几度拖" */
    if (follow_on != 0u)
    {
        float e   = alpha_deg;
        float ae  = (e > 0.0f) ? e : -e;
        float dir = (e > 0.0f) ? 1.0f : -1.0f;

        ae -= RC_FOLLOW_DEAD;               /* 死区外才纠，边界处输出恰为 0 */

        if (ae > 0.0f)
        {
            float ramp = (ae > RC_FOLLOW_DEAD) ? 1.0f : (ae / RC_FOLLOW_DEAD);
            float mag  = RC_FOLLOW_KP * ae + RC_FOLLOW_KICK * ramp;

            /* 阻尼：α̇ 直接用编码器转速 6·rpm（可测、无滤波、无相位代价）。
             * α̇ 与 e 反号 = 正在回来 → 收油；同号 = 还在往外跑 → 加大。
             * 原来的纯 P 没有这一项，冲过头再回来就是"摆"。 */
            mag += RC_FOLLOW_KD * (dir * 6.0f * (float)gim6020.rpm);
            if (mag < 0.0f) mag = 0.0f;

            spin = RC_FOLLOW_SIGN * dir * mag;
        }
        else
        {
            spin = 0.0f;
        }

        if (spin >  RC_FOLLOW_MAX) spin =  RC_FOLLOW_MAX;
        if (spin < -RC_FOLLOW_MAX) spin = -RC_FOLLOW_MAX;
    }
#endif

   /* 跟随/小陀螺的自转指令是闭环算出来的（常常 < 0.02），必须旁路混控的归一化死区，
    * 否则 3.1° 以内指令全被清零：起转有门槛、到位后一顿一顿 */
   Mecanum_SetSpinDeadbandBypass((uint8_t)((follow_on != 0u) || (spin_mode != 0u)));

   uint8_t mix_state = Mecanum_Mix(fwd, strafe, spin, enabled, RC_WheelOut);

   if(!online && ((now - RC_Data.last_tick) > RC_FAILSAFE_MS))
{
    for(uint8_t i = 0 ; i < MECANUM_WHEEL_NUM; i++)
    {
        RC_WheelOut[i] = 0.0f;
    }
   
    Mecanum_Reset();
    RC_Wheel_Reset();
    mix_state = 2u;
}

static const float s_wheel_sign[MECANUM_WHEEL_NUM] ={
    MOTOR_POL_ID1, MOTOR_POL_ID2, MOTOR_POL_ID3, MOTOR_POL_ID4};

     float max_a = (float )RC_MAX_CURRENT_A;
     int16_t  raw[MECANUM_WHEEL_NUM];
     uint8_t  fault_mask = 0u;


    for (uint8_t i = 0 ; i < MECANUM_WHEEL_NUM; i++)
    {
        raw[i] = 0;
        RC_WheelRef[i] = 0.0f;      /* 断使能/故障时不残留上一拍目标值 */
        
        if((HAL_GetTick() - dj_wheel[i].last_tick) > RC_WHEEL_FEEDBACK_MS)
         {
                fault_mask |= (uint8_t)(1u << i);
         }
        if(dj_wheel[i].temp > RC_WHEEL_TEMP_LIMIT)
         {
                fault_mask |= (uint8_t)(1u << i);
         }

    }
      RC_WheelFault = fault_mask;

      if(mix_state == 2u || (fault_mask != 0u && RC_WHEEL_FAULT_ISOLATE == 0))
    {
      RC_Wheel_Reset();
      for(uint8_t i = 0 ; i < MECANUM_WHEEL_NUM; i++)
      {
          raw[i] = 0;
          RC_WheelRef[i] = 0.0f;
      }

    }
    else if ((mix_state == 1u) && (follow_on == 0u) && (spin_mode == 0u))
    {
      /* 整车回中：当拍四轮积分与目标归零，跳过普通 PI；
       * 用实测转速做纯 P 反刹（恒逆着运动方向），停稳后撤掉电流。
       * 跟随/小陀螺模式下自转指令是环算出来的，会频繁穿过 0，
       * 这时清斜坡等于每穿一次就从零重加速 → 保留斜坡，只给零电流 */
      if ((follow_on == 0u) && (spin_mode == 0u))
      {
          Mecanum_Reset();                /* 手动模式：旧斜坡一并清掉，重新推杆从零起步 */
      }
      for (uint8_t i = 0; i < MECANUM_WHEEL_NUM; i++)
      {
          float rpm = (float)dj_wheel[i].rpm;
          float a;

          s_wheel_i[i]   = 0.0f;
          RC_WheelRef[i] = 0.0f;

          /* 故障轮：反馈已停更，它的 rpm 是最后一帧的旧值，
           * 拿它算刹车会一直往已停的轮子上灌逆向电流 → 直接给 0 */
          if ((fault_mask & (uint8_t)(1u << i)) != 0u)
          {
              raw[i] = 0;
              continue;
          }

          if (wheel_abs(rpm) < RC_WHEEL_STOP_QUIET_RPM)
          {
              a = 0.0f;
          }
          else
          {
              a = wheel_clamp(-RC_WHEEL_STOP_KP * rpm, -RC_WHEEL_STOP_A, RC_WHEEL_STOP_A);
          }

          raw[i] = (int16_t)(a / 20.0f * 16384.0f);
      }
    }
    else {

      for(uint8_t i = 0; i < MECANUM_WHEEL_NUM; i++)
     {
       if(RC_WHEEL_FAULT_ISOLATE == 1 && (fault_mask & (uint8_t)(1u << i)))
       {
          s_wheel_i[i] = 0.0f;
          raw[i] = 0;
          continue;
       }

      float  ref = s_wheel_sign[i] * RC_WheelOut[i] * RC_WHEEL_RPM_MAX;
      float  a;

      RC_WheelRef[i] = ref;      /* 调试镜像：该轮目标转速 */

      #if (RC_WHEEL_ENABLE_PID ==1)
        a = RC_Wheel_Speed_PI(i, ref, dj_wheel[i].rpm, max_a);
      #else 
        (void)ref;
        a = s_wheel_sign[i] * RC_WheelOut[i] * max_a;
      #endif

      if(a > RC_ABS_CURRENT_A) a = RC_ABS_CURRENT_A;
      if(a < -RC_ABS_CURRENT_A) a = -RC_ABS_CURRENT_A;

        raw[i] = (int16_t)(a / 20.0f * 16384.0f);

     }
    }

    /* 6) 云台 GM6020（yaw）：限位 + 超速兜底 + 斜坡 + 安全开关
     *    小陀螺走专用世界航向环，其余模式走原来的手控/跟随环，两条路互不干扰 */
    float gim  = online ? RC_Norm(RC_CH_GIMBAL) : 0.0f;
    float volt = (spin_mode != 0u) ? RC_Yaw_Spin_Control(online, gim)
                                   : RC_Yaw_Control(online, gim);

    /* ① 反馈超时 / 遥控离线 → 断电（没有反馈就等于没有限位） */
    uint8_t yaw_kill = 0u;
    if (online == 0u)
        yaw_kill = 1u;
    if ((HAL_GetTick() - gim6020.last_tick) > GM_FEEDBACK_TIMEOUT_MS)
        yaw_kill = 1u;

    /* ② 超速兜底：不管角度算得对不对，转太快就断 */
    if (gim6020.rpm > GM_RPM_LIMIT || gim6020.rpm < -GM_RPM_LIMIT)
        yaw_kill = 1u;

    /* ④ 斜坡：每拍最多变 GM_VOLT_SLEW，避免扭矩冲击
     *    故障时不能走斜坡——否则上一拍 18000，这一拍只是 17800，
     *    要 90 ms 才掉到 0，等于故障期间还在给扭矩。直接把斜坡状态也清掉。 */
    static float volt_prev = 0.0f;
    if (s_yaw_slew_kick != 0u) {
        s_yaw_slew_kick = 0u;
        volt_prev = 0.0f;       /* 交接后从 0 重新爬，不吃上一模式的旧斜坡 */
    }
    if (yaw_kill != 0u) {
        volt      = 0.0f;
        volt_prev = 0.0f;       /* 恢复时从 0 重新往上爬 */
    } else {
        if (volt > volt_prev + GM_VOLT_SLEW) volt = volt_prev + GM_VOLT_SLEW;
        if (volt < volt_prev - GM_VOLT_SLEW) volt = volt_prev - GM_VOLT_SLEW;
    }
    RC_YawSpinOut = volt;       /* 实发电压（斜坡之后），小陀螺环的积分回算用它 */

     /*pitch电机赋值*/
      float pit  = online ? RC_Norm(RC_CH_GIMBAL_PITCH) : 0.0f;
      float volt_p = RC_Pitch_Control(online, 1u, pit);

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
    else//若未给值即拨杆不在上
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
    RC_YawVoltOut = volt;       /* 斜坡之后的真实发送值（RC_YawVolt 是斜坡之前） */

    /* ⑥ 发送：CAN2 = 底盘四轮 + yaw；CAN1 = 摩擦轮 + pitch */
#if (FRIC_USE_CAN1)
  #if (MECANUM_WHEEL_NUM == 4)
    DJ3508_Send_I_Can2(raw[0], raw[1], raw[2], raw[3]); /* CAN2：四轮 0x200 槽位全用 */
 
  #endif
    DJ6020_Send_V_Can2((int16_t)volt, 0);       /* CAN2：yaw ID1 */
    DJ3508_Send_Can1(fric_l, fric_r);           /* CAN1：摩擦轮 ID2 / ID3 */
    DJ6020_Send_Can1((int16_t)volt_p);          /* CAN1：pitch ID2 */

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























