/**
 * @file    gimbal_spin.c
 * @brief   小陀螺（世界系）yaw 环 + 模式交接
 * @note    2026-11-19 分层整理：以下内容从 remote/rc_dbus.c 原样搬迁，逻辑未改。
 *          改动仅限：文件头注释、include、以及跨文件共享变量的存储类。
 */
#include "rc_dbus.h"
#include "rc_state.h"
#include "usart.h"
#include "DJ_Motor.h"
#include "mecanum.h"
#include "ins.h"
#include "fdcan.h"
#include "bsp_can.h"
#include <string.h>
#include <math.h>

/* 小陀螺专用环的状态（与 yaw 环互不共享，靠 RC_Yaw_Handover 交接）
 * s_yaw_slew_kick 原先 static，分层后 robot.c 要读，改全局并由 rc_state.h 声明 */
static float   s_spin_ref   = 0.0f;  /* 世界航向目标 deg（多圈连续） */
static float   s_spin_i     = 0.0f;  /* 速度环积分（电压单位） */
static uint8_t s_spin_ready = 0u;
static uint16_t s_spin_bad  = 0u;   /* 失控计数（WorldYawRate 长时间超限） */
uint8_t  s_yaw_slew_kick = 0u;  /* 交接后把输出斜坡清零的标志 */
volatile float RC_SpinYaw = 0.0f;
volatile float RC_SpinWch = 0.0f;
volatile float RC_SpinErr = 0.0f;
volatile float RC_YawSpinOut = 0.0f;   /* 小陀螺环上一拍真正发出去的电压（斜坡+限幅之后），供积分回算 */
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
