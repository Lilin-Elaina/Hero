/**
 * @file    gimbal.c
 * @brief   云台 pitch 双环 + yaw 车体系角度环 + 枪口夹角
 * @note    2026-11-19 分层整理：以下内容从 remote/rc_dbus.c 原样搬迁，逻辑未改。
 *          改动仅限：文件头注释、include、以及跨文件共享变量的存储类。
 */
#include "gimbal.h"
#include "rc_dbus.h"
#include "bsp_time.h"
#include "rc_state.h"
#include "DJ_Motor.h"
#include "mecanum.h"
#include "ins.h"
#include "bsp_can.h"
#include <string.h>
#include <math.h>

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
/* 非 static：分层后由 application/robot.c 的 1 kHz 编排调用 */
float RC_Pitch_Control(uint8_t online, uint8_t armed, float stick)
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


 if ((BSP_Now() - gim_pitch.last_tick) > GM_FEEDBACK_TIMEOUT_MS ||
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
    s_home_since = BSP_Now() | 1u;     /* 接管时记一次起点，供回中计时用 */
    s_stage = ( GM_PITCH_HOME_ENABLE && !s_homed) ? 1 : 2;
    return 0.0f;
}

/*在线之后进行判断，若未回中，开始回中*/
if(s_stage == 1)
{
    s_v = 0.0f;
    /*若一直没符合下面置2的三个条件时触发*/
    if ((BSP_Now() - s_home_since) > GM_PITCH_HOME_TIMEOUT_MS)
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
/* ======================= 云台 yaw 角度闭环 ======================= */
/* 前三个原来是文件级 static；分层后 gimbal_spin.c 的交接函数也要读写，
 * 改为全局并由 rc_state.h 声明（只改存储类，类型/初值/语义不变） */
float   s_yaw_ref   = 0.0f;   /* 目标角（counts） */
float   s_yaw_i     = 0.0f;   /* 速度环积分（电压单位） */
uint8_t s_yaw_ready = 0u;
static float   s_psi_deg   = 0.0f;   /* 枪口世界航向角（度，陀螺积分） */
static float   s_psi_ref   = 0.0f;   /* 航向锁存基准（度） */
static float   s_hold_i    = 0.0f;   /* 稳向积分（deg/s），只在底盘带着转时累加 */

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
    const uint8_t fresh = ((BSP_Now() - gim6020.last_tick) <= GM_FEEDBACK_TIMEOUT_MS) ? 1u : 0u;
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
