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
volatile float RC_YawVolt   = 0.0f;   /* 最终电压指令 */
volatile float RC_YawI      = 0.0f;   /* 速度环积分 */

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
    /* 反馈失效时不能拿最后一帧冻结的角度当 α：
     * 那份角度不再变化，跟随环会以为"还差这么多"，底盘就一直转下去。
     * 这时按"枪口对正车头"处理（α=0），车仍能按车体系行驶。 */
    if ((HAL_GetTick() - gim6020.last_tick) > GM_FEEDBACK_TIMEOUT_MS) {
        RC_GunAngle = 0.0f;
        return 0.0f;
    }

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
    uint8_t s_ins_fresh = 0u;
    float d_ref, err, rpm_d, err_v, volt;

    /* IMU 采样新鲜度：INS_Sample 每成功采一帧 +1，20 ms 不动 = SPI 挂了。
     * 读失败时 INS_Gyro[] 会停在最后一帧，拿它做稳向补偿等于一直补一个假转速。 */
    {
        static uint32_t s_i_cnt = 0u;
        static uint32_t s_i_tick = 0u;
        uint32_t t_now = HAL_GetTick();

        if (INS_Sample != s_i_cnt) {
            s_i_cnt  = INS_Sample;
            s_i_tick = t_now;
        }
        if ((INS_Ready != 0u) && ((t_now - s_i_tick) <= 20u)) {
            s_ins_fresh = 1u;
        }
    }

    if ((online == 0u) || (fresh == 0u) || (s_yaw_ready == 0u)) {
        s_yaw_ref   = cur;
        s_yaw_i     = 0.0f;
        s_yaw_ready = (uint8_t)((online != 0u) && (fresh != 0u));
        RC_YawRef    = s_yaw_ref;
        RC_YawErr    = 0.0f;
        RC_YawRpmDes = 0.0f;
        RC_YawVolt   = 0.0f;
        RC_YawI      = s_yaw_i;
        return 0.0f;
    }

    /* ① 摇杆 → 目标角增量（松杆保持）
     * 加一个加速度限制：原来松杆瞬间 d_ref 从满值直接掉到 0，
     * 位置目标（积分项）和速度前馈同时阶跃，制动冲击很大。
     * 这里限制每拍的变化量，满杆约 100 ms 加满 / 松掉。 */
    {
        static float s_dref = 0.0f;
        float dref_tgt = GM6020_SIGN * GM_YAW_STICK_RATE * stick * 0.001f;
        float dd = dref_tgt - s_dref;

        if (dd >  GM_YAW_STICK_ACCEL) dd =  GM_YAW_STICK_ACCEL;
        if (dd < -GM_YAW_STICK_ACCEL) dd = -GM_YAW_STICK_ACCEL;

        s_dref += dd;
        d_ref   = s_dref;
    }
    s_yaw_ref += d_ref;

    /* ①.2 松杆就不攒积分
     * 云台 yaw 是水平轴，没有重力要顶；松杆后位置环的 P 项足够把角度按住。
     * 积分只在"主动推杆"时有用，松杆还留着它，就会慢慢把枪口顶偏再弹回来。 */
    if (d_ref == 0.0f) {
        s_yaw_i = 0.0f;
    }

    /* ①.5 底盘转动补偿（稳向）
     * 问题：目标角是「相对车头」的，车头一转，枪口就跟着转。
     * 做法：车体转多少，目标角就反向退多少 → 枪口在世界里不动。
     * 只在「确实在转」时积分，静止完全不积分 → 没有零偏漂移。 */
    {
        /* IMU 绑在枪架上（跟着 yaw/pitch 一起动），INS_Gyro[0] 读到的是
         * 「枪口」角速度 = 底盘角速度 + 关节角速度。
         * 稳向只该补底盘那一份。
         * 关节角速度必须用「实测」而不是摇杆指令：松杆那一拍指令已经归零，
         * 云台还在转，用指令会把这段余速误当成底盘在转，把目标反向拉走
         * （一边制动、一边被目标拽回来）。
         * 实测 rpm 有 1 rpm(=6°/s) 的量化台阶，先低通再减；
         * 操作手正在快转（|关节角速度| 超过 GM_YAW_STAB_JOINT_MAX）时直接让位，
         * 那一小段本来就是他手动在控，稳向不掺和。 */
        float wz_muzzle = -(float)INS_Gyro[0] * 57.2957795f;   /* 枪口角速度 deg/s */
        float wz_joint  = 0.0f;
        uint8_t stab_ok = 1u;
#if (GM_YAW_STAB_JOINT_FF)
        {
            static float s_joint_dps = 0.0f;

            s_joint_dps += ((float)gim6020.rpm * 6.0f - s_joint_dps) * GM_YAW_JOINT_LPF;
            wz_joint = s_joint_dps;

            if ((wz_joint > GM_YAW_STAB_JOINT_MAX) || (wz_joint < -GM_YAW_STAB_JOINT_MAX)) {
                stab_ok = 0u;
            }
        }
#endif
        float wz_chassis = wz_muzzle - wz_joint;               /* 底盘角速度 deg/s */

        /* IMU 采样停了就不要再积分：INS_Gyro[] 会停在最后一帧，越积越偏 */
        if ((stab_ok != 0u) && (s_ins_fresh != 0u) &&
            (wz_chassis > GM_YAW_STAB_DEAD || wz_chassis < -GM_YAW_STAB_DEAD)) {
            s_yaw_ref -= GM_YAW_STAB_SIGN * wz_chassis * GM_YAW_COUNT_PER_DEG * 0.001f;
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

    rpm_d = GM_YAW_POS_DIR * GM_YAW_ANG_KP * err
          + d_ref * 1000.0f / 136.53f
          - GM_YAW_POS_DIR * GM_YAW_ANG_KD * (float)gim6020.rpm;   /* D：临近目标先减速，别冲过去 */
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

    /* 自动运动（跟随/小陀螺）必须有可信反馈：云台角、IMU、四轮 任一路失效就退出。
     * 只关这两个模式，底盘仍可手推，不会整车瘫掉。 */
    {
        uint32_t t_now = HAL_GetTick();
        uint8_t  yaw_ok   = ((t_now - gim6020.last_tick) <= GM_FEEDBACK_TIMEOUT_MS) ? 1u : 0u;
        uint8_t  ins_ok   = ((INS_Ready != 0u) && (INS_CalDone != 0u)) ? 1u : 0u;
        uint8_t  wheel_ok = 1u;
        uint8_t  wi;

        for (wi = 0u; wi < MECANUM_WHEEL_NUM; wi++) {
            if ((t_now - dj_wheel[wi].last_tick) > RC_WHEEL_FEEDBACK_MS) {
                wheel_ok = 0u;
            }
        }

        /* 采样停摆（SPI 读失败）等同于没有 IMU */
        {
            static uint32_t s_i_cnt = 0u;
            static uint32_t s_i_tick = 0u;

            if (INS_Sample != s_i_cnt) {
                s_i_cnt  = INS_Sample;
                s_i_tick = t_now;
            }
            if ((t_now - s_i_tick) > 20u) {
                ins_ok = 0u;
            }
        }

        if ((yaw_ok == 0u) || (ins_ok == 0u) || (wheel_ok == 0u)) {
            follow_on = 0u;
            spin_mode = 0u;
        }
    }
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
     * 只用云台编码器，零漂、不积分。
     * 结构 = 死区 + 比例 P + 固定起步量 KICK + 限幅，没有 I 项：
     * 纯 P 在最后几度只剩几十分之一的指令、推不动车，所以加一个常数起步量；
     * 若出现"转一下、停一下、又转"，先调 KICK 和轮速响应，别一味加 KICK。 */
    if (follow_on != 0u)
    {
        float e = alpha_deg;

        if      (e >  RC_FOLLOW_DEAD) e -= RC_FOLLOW_DEAD;
        else if (e < -RC_FOLLOW_DEAD) e += RC_FOLLOW_DEAD;
        else                          e  = 0.0f;

        if (e != 0.0f)
        {
            /* 起步量 + 比例：纯比例的尾巴只剩几十分之一的指令，推不动车 → 拖 */
            float mag = RC_FOLLOW_KP * ((e > 0.0f) ? e : -e) + RC_FOLLOW_KICK;

            /* 起步量 + 比例；RC_FOLLOW_SIGN 之前定义了却没参与输出，已接上 */
            spin = RC_FOLLOW_SIGN * ((e > 0.0f) ? 1.0f : -1.0f) * mag;
        }
        else
        {
            spin = 0.0f;
        }

        if (spin >  RC_FOLLOW_MAX) spin =  RC_FOLLOW_MAX;
        if (spin < -RC_FOLLOW_MAX) spin = -RC_FOLLOW_MAX;
    }
#endif

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
    else if (mix_state == 1u)
    {
      /* 整车回中：当拍四轮积分与目标归零，跳过普通 PI；
       * 用实测转速做纯 P 反刹（恒逆着运动方向），停稳后撤掉电流 */
      Mecanum_Reset();                    /* 旧斜坡一并清掉，重新推杆从零起步 */
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

    /* 6) 云台 GM6020（yaw）：限位 + 超速兜底 + 斜坡 + 安全开关 */
    float gim  = online ? RC_Norm(RC_CH_GIMBAL) : 0.0f;
    float volt = RC_Yaw_Control(online, gim);

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
    if (yaw_kill != 0u) {
        volt      = 0.0f;
        volt_prev = 0.0f;       /* 恢复时从 0 重新往上爬 */
    } else {
        if (volt > volt_prev + GM_VOLT_SLEW) volt = volt_prev + GM_VOLT_SLEW;
        if (volt < volt_prev - GM_VOLT_SLEW) volt = volt_prev - GM_VOLT_SLEW;
    }

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























