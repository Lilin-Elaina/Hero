/**
 * @file    robot.c
 * @brief   1 kHz 控制编排（RC_Control_Task）与全局调试镜像定义
 * @note    2026-11-19 分层整理：以下内容从 remote/rc_dbus.c 原样搬迁，逻辑未改。
 *          改动仅限：文件头注释、include、以及跨文件共享变量的存储类。
 */
#include "rc_dbus.h"
#include "bsp_time.h"
#include "rc_state.h"
#include "usart.h"
#include "DJ_Motor.h"
#include "mecanum.h"
#include "ins.h"
#include "fdcan.h"
#include "bsp_can.h"
#include <string.h>
#include <math.h>

/* 最近一次 RC_Control_Task 算出的各轮归一化目标（调试用，可在调试器里看） */
float RC_WheelOut[MECANUM_WHEEL_NUM] = {0};

/* 使能三态机：0=等待条件 1=计时中（软启动门） 2=已使能 */
typedef enum { RC_EN_WAIT = 0, RC_EN_ARMING = 1, RC_EN_READY = 2 } RC_EnableState_t;

volatile uint8_t  RC_EnableState = RC_EN_WAIT;   /* 调试镜像 */
volatile uint32_t RC_EnableMs    = 0u;           /* 计时中已等了多久 ms */

/* 调试镜像：yaw 闭环内部量（只读，给 Ozone 看） */
volatile float RC_YawRef    = 0.0f;   /* 目标角 counts */
volatile float RC_YawErr    = 0.0f;   /* 角度误差 counts */
volatile float RC_YawRpmDes = 0.0f;   /* 位置环输出的目标转速 rpm */
volatile float RC_YawVolt   = 0.0f;   /* 斜坡之前的电压指令 */
volatile float RC_YawVoltOut= 0.0f;   /* 斜坡之后、真正发出去的电压 */
volatile float RC_YawI      = 0.0f;   /* 速度环积分 */
volatile float RC_HoldWz    = 0.0f;   /* 稳向看到的枪口世界角速度 deg/s */
volatile float RC_HoldRate  = 0.0f;   /* 稳向折算出的补偿角速度 deg/s */

/* ---- 掉线 / yaw 断电取证镜像（定义见 rc_dbus.h 注释）---- */
volatile uint32_t RC_FrameCnt    = 0;
volatile uint32_t RC_GapMaxMs    = 0;
volatile uint32_t RC_ErrCnt      = 0;
volatile uint32_t RC_YawFbAgeMax = 0;
volatile float    RC_YawSpdMax   = 0.0f;
volatile uint8_t  RC_YawKill     = 0u;
volatile uint32_t RC_YawKillCnt  = 0;
volatile uint32_t RC_Can2TEC     = 0;
volatile uint32_t RC_Can2REC     = 0;
volatile uint32_t RC_Can2TECMax  = 0;
volatile uint32_t RC_Can2RECMax  = 0;
volatile uint32_t RC_YawOutageCnt = 0;
/* ---- CAN2 控制器状态镜像（比 TEC/REC 更能区分“线断了”和“有人应答失败”）----
 * PSR.LEC 最后错误码：0=无错 1=填充错 2=格式错 3=ACK错 4=位1错 5=位0错 6=CRC错
 *   → 3(ACK)  = 我们发出去了但没有任何节点应答（线断/对端全掉电）
 *   → 4/5(位错)= 线上电平与预期不符（短路/终端电阻/收发器坏）
 *   → 1/2/6    = 干扰/反射（终端电阻、屏蔽、线长）
 * CCCR.INIT=1 → 控制器停在初始化态（Stop 后未重启，或 Bus-Off 恢复失败）*/
volatile uint32_t RC_Can2PSR     = 0;
volatile uint32_t RC_Can2CCCR    = 0;
volatile uint32_t RC_Can2RestartCnt = 0;  /* RX 活性看门狗触发的 Stop+Start 次数 */

/* ------------------------------------------------------------------ */
/* 阶段函数的文件级状态（原先是函数内 static，提取后语义完全等价）      */
/* ------------------------------------------------------------------ */
/* 每轮电流指令的极性：D 之后的车体四个轮子 */
static const float s_wheel_sign[MECANUM_WHEEL_NUM] = {
    MOTOR_POL_ID1, MOTOR_POL_ID2, MOTOR_POL_ID3, MOTOR_POL_ID4};

static float s_yaw_volt_prev = 0.0f;   /* yaw 电压斜坡状态 */
static float s_fric_a        = 0.0f;   /* 摩擦轮电流斜坡状态 */

/* 1 kHz 控制拆成四段：底盘 → 云台 → 摩擦轮，顺序与拆分前逐拍一致 */
static void RC_Wheel_Step(uint8_t mix_state, uint8_t follow_on, uint8_t spin_mode,
                          int16_t *raw);
static void RC_Yaw_Step(uint32_t now, uint8_t online, uint8_t spin_mode, float *volt);
static void RC_Fric_Step(uint8_t online, int16_t *fric_l, int16_t *fric_r);


/* ★固件版本指纹（Watch 自检用）：
 * Ozone 里读 RC_BuildTag，必须等于下面这个值；读出来是 0/乱数 = 挂的 ELF 与芯片
 * 固件不是同一版（地址全错位，Watch 里所有变量读数都不可信）。
 * 每次改了全局变量布局/新增变量就把后 4 位改一下，方便确认。 */
volatile uint32_t RC_BuildTag = 0x1117235Du;   /* 0x…5D = 2026-11-20 失联判据改“连续丢帧”+ 失联即停车 */
/* ------------------------------------------------------------------ */
/* 控制任务：1kHz 调用一次                                              */
/*   摇杆 → 归一化 → 麦轮混控(含斜坡) → 限幅 → CAN 电流指令              */
/* ------------------------------------------------------------------ */
void RC_Control_Task(void)
{     /*通信在线判断*/
    uint32_t now = BSP_Now();

    uint8_t online = RC_IsOnline(now);

    /* ---- CAN2 总线健康快照（每拍一次，读 ECR 寄存器无副作用）----
     * TEC/REC 是 FDCAN 的错误计数器：>127 = 错误被动（收发开始拖延迟重试，
     * 表现就是整车“游戏延迟”）；255 = 接近/已 bus-off。
     * 之前这只能拿 J-Link 手动读寄存器，现在出问题当场在 Ozone 里定罪。 */
    {
        uint32_t ecr = hfdcan2.Instance->ECR;
        uint32_t tec = ecr & 0xFFu;
        uint32_t rec = (ecr >> 8) & 0x7Fu;

        RC_Can2TEC = tec;
        RC_Can2REC = rec;
        if (tec > RC_Can2TECMax) RC_Can2TECMax = tec;
        if (rec > RC_Can2RECMax) RC_Can2RECMax = rec;
        RC_Can2PSR  = hfdcan2.Instance->PSR;
        RC_Can2CCCR = hfdcan2.Instance->CCCR;

        /* 版本指纹：每拍写入常量（同时保证该变量不被 --gc-sections 回收）。
         * Watch 读到的值 ≠ 0x1117235D → 挂的 ELF 与芯片固件不是同一版，其余读数全部不可信 */
        RC_BuildTag = 0x1117235Du;
    }

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

    uint8_t enabled = 0u;

    /* 使能门：拨杆到位/失联恢复后必须“连续保持 RC_ENABLE_DELAY_MS”才放行。
     * 旧版是 if/else 两拍写法，有一个实实在在的 bug：
     *   enabled 第一次为真的那一拍只记录起始时刻，**当拍并没有清 enabled**，
     *   于是电机先拿满指令、下一拍才被门挡住 → 手感是“点一下、停 100ms、再上电”。
     * 现在改成显式三态机，等待期一定输出 0，且等待期内只要条件掉一次就重新计时。 */
    {
        static uint8_t  s_en_state = RC_EN_WAIT;
        static uint32_t s_en_since = 0u;

        const uint8_t req = (uint8_t)((online != 0u) && (armed != 0u));

        if (req == 0u)
        {
            s_en_state = RC_EN_WAIT;        /* 条件不成立：立即撤销，并清计时 */
            s_en_since = 0u;
            enabled    = 0u;
        }
        else
        {
            switch (s_en_state)
            {
            case RC_EN_WAIT:
                s_en_state = RC_EN_ARMING;  /* 刚满足：开始计时，当拍仍不给力 */
                s_en_since = now;
                enabled    = 0u;
                break;

            case RC_EN_ARMING:
                if ((now - s_en_since) >= RC_ENABLE_DELAY_MS)
                {
                    s_en_state = RC_EN_READY;
                    enabled    = 1u;
                }
                else
                {
                    enabled = 0u;
                }
                break;

            case RC_EN_READY:
            default:
                enabled = 1u;
                break;
            }
        }

        RC_EnableState = s_en_state;        /* 调试镜像：0=等待 1=计时中 2=已使能 */
        RC_EnableMs    = (s_en_state == RC_EN_ARMING) ? (now - s_en_since) : 0u;
    }

    /* ---- CAN2 RX 活性看门狗（2026-11-18 加，2026-11-19 改判据）----
     * 底盘四轮 + yaw 全在 CAN2，五个节点每个 1kHz 反馈。实测出现过整条 CAN2
     * 静默 337 秒不恢复：唯一能解释“死透且无人再试”的软件路径 = Bus-Off 回调里
     * Stop+Start 万一没救回来（ISR 内重启 HAL 状态机有竞态），控制器就永远卡在
     * CCCR.INIT，而全工程没有任何代码会再试第二次。
     *
     * 判据用 yaw 6020 的反馈时间戳 gim6020.last_tick（每收到一帧 0x205 就更新），
     * 不再依赖“帧数计数器”——计数器删掉了看门狗也不会跟着失效：
     *   - 距上一帧 >= 20ms → Stop + FDCAN2_Start 重启一次
     *   - 之后每 100ms 再试一次，直到恢复
     *   - RC_Can2RestartCnt 计数留证：重启后立刻恢复 = 卡死确系软件；
     *     重启多少次都没用 = 总线物理上真的死了（回头查硬件）
     *   - 上电后必须先见过至少一帧（last_tick != 0）且已使能才启用，
     *     避免台架上不带电调时不停空转重启 */
    {
        /* 最近一次收到 CAN2 任意节点反馈的时刻：五个节点（四轮 + yaw）取最新。
         * 只要还有一个节点在回帧，就说明总线是活的，不重启——否则单个节点
         * 掉线（或某个 C620 拨码跳到 ID5 冒充 0x205）会让我们每秒重启十次总线，
         * 反而把好好的四轮打断。 */
        uint32_t last_rx = gim6020.last_tick;

        for (uint8_t i = 0u; i < MECANUM_WHEEL_NUM; i++)
        {
            if ((int32_t)(dj_wheel[i].last_tick - last_rx) > 0)
            {
                last_rx = dj_wheel[i].last_tick;
            }
        }

        static uint8_t s_can2_seen = 0u;   /* 上电后是否见过 CAN2 帧 */

        if (last_rx != 0u)
        {
            s_can2_seen = 1u;
        }

        if ((s_can2_seen != 0u) && (enabled != 0u))
        {
            int32_t age = (int32_t)(now - last_rx);   /* 距最近一帧的毫秒数 */

            /* ISR 可能在本拍取到 now 之后才更新 last_tick，差值为负时按 0 处理 */
            if (age < 0)
            {
                age = 0;
            }

            /* age 达到 20 之后每 100ms 触发一次（20/120/220…），
             * 用 age 直接取模，省掉一个“冻结计数器” */
            if ((age >= 20) && (((age - 20) % 100) == 0))
            {
                RC_Can2RestartCnt++;
                BSP_CAN_Stop(2u);          /* bsp 层接口：上层不再直接调 HAL_FDCAN */
                FDCAN2_Start();
            }
        }
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

     int16_t  raw[MECANUM_WHEEL_NUM];

     RC_Wheel_Step(mix_state, follow_on, spin_mode, raw);

    float volt;

    RC_Yaw_Step(now, online, spin_mode, &volt);

     /*pitch电机赋值*/
      float pit  = online ? RC_Norm(RC_CH_GIMBAL_PITCH) : 0.0f;
      float volt_p = RC_Pitch_Control(online, 1u, pit);

    int16_t fric_l;
    int16_t fric_r;

    RC_Fric_Step(online, &fric_l, &fric_r);

    /* ⑤ 安全开关：yaw / pitch / 摩擦轮 全部清零 */
#if GM_SAFE_NO_OUTPUT
    volt   = 0.0f;
    volt_p = 0.0f;
    fric_l = 0;
    fric_r = 0;
#endif
    s_yaw_volt_prev = volt;     /* ★ 记下本拍输出，下一拍才能继续爬（斜坡状态在阶段 3） */
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
/* 阶段 2：底盘四轮 → 电流指令（原 RC_Control_Task 的第 5 段）          */
/* ------------------------------------------------------------------ */
/* 失联判据用“连续丢帧计数”而不是“单次超时”：
 * 碰撞时线束抖一下就会让某帧晚到几十毫秒，旧判据（>100 ms 即隔离）会把
 * 那只轮子直接断电、另外三轮继续推 → 整车以它当支点打转。
 * 现在：连续 RC_WHEEL_FAULT_MISS 拍没有新帧才判失联，恢复也要连续
 * RC_WHEEL_FAULT_CLEAR 拍好帧（滞回），避免在临界点上反复抖。
 * 判成失联就整车停车（故障轮断电 + 其余三轮反向制动），不再“少一条腿满力跑”。 */
static uint16_t s_wheel_miss[MECANUM_WHEEL_NUM];
static uint32_t s_wheel_seen[MECANUM_WHEEL_NUM];
static uint16_t s_wheel_ok[MECANUM_WHEEL_NUM];
static uint8_t  s_wheel_lost[MECANUM_WHEEL_NUM];

static void RC_Wheel_Step(uint8_t mix_state, uint8_t follow_on, uint8_t spin_mode,
                          int16_t *raw)
{
     float   max_a = (float )RC_MAX_CURRENT_A;
     uint8_t fault_mask = 0u;
     uint8_t lost_mask  = 0u;

    for (uint8_t i = 0 ; i < MECANUM_WHEEL_NUM; i++)
    {
        raw[i] = 0;
        RC_WheelRef[i] = 0.0f;      /* 断使能/故障时不残留上一拍目标值 */

        /* 连续丢帧计数 + 滞回恢复 */
        if (dj_wheel[i].last_tick != s_wheel_seen[i])
        {
            s_wheel_seen[i] = dj_wheel[i].last_tick;
            s_wheel_miss[i] = 0u;
            if (s_wheel_ok[i] < 0xFFFFu)
            {
                s_wheel_ok[i]++;
            }
            if (s_wheel_ok[i] >= RC_WHEEL_FAULT_CLEAR)
            {
                s_wheel_lost[i] = 0u;       /* 连续 CLEAR 拍好帧才恢复 */
            }
        }
        else
        {
            s_wheel_ok[i] = 0u;
            if (s_wheel_miss[i] < 0xFFFFu)
            {
                s_wheel_miss[i]++;
            }
            if ((s_wheel_miss[i] >= RC_WHEEL_FAULT_MISS) ||
                ((BSP_Now() - dj_wheel[i].last_tick) > RC_WHEEL_FEEDBACK_MS))
            {
                s_wheel_lost[i] = 1u;
            }
        }

        if (s_wheel_lost[i] != 0u)
        {
            lost_mask |= (uint8_t)(1u << i);
        }
        if (dj_wheel[i].temp > RC_WHEEL_TEMP_LIMIT)
        {
            fault_mask |= (uint8_t)(1u << i);   /* 过温只隔离该轮，不整车停 */
        }

    }
      fault_mask |= lost_mask;
      RC_WheelFault = fault_mask;

      /* ★失联即停车：故障轮断电，其余三轮反向制动把车刹住。
       *   让“少一条腿”的车继续推，只会以故障轮为支点打转，比停更难控。 */
      if (lost_mask != 0u)
      {
          RC_Wheel_Reset();
          Mecanum_Reset();

          for (uint8_t i = 0; i < MECANUM_WHEEL_NUM; i++)
          {
              float rpm = (float)dj_wheel[i].rpm;
              float a   = 0.0f;

              RC_WheelRef[i] = 0.0f;

              if ((lost_mask & (uint8_t)(1u << i)) != 0u)
              {
                  raw[i] = 0;           /* 没有反馈，绝不能凭最后一帧旧值给电流 */
                  continue;
              }
              if (wheel_abs(rpm) >= RC_WHEEL_STOP_QUIET_RPM)
              {
                  float brake = RC_WHEEL_STOP_KP * wheel_abs(rpm) + RC_WHEEL_STOP_KP0;
                  a = (rpm > 0.0f) ? -brake : brake;
                  a = wheel_clamp(a, -RC_WHEEL_STOP_A, RC_WHEEL_STOP_A);
              }
              raw[i] = (int16_t)(a / 20.0f * 16384.0f);
          }
          return;
      }

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
              /* 比例项 + 恒定项：只靠 -KP*rpm 时，转速越低制动越小，
               * 架空测试（几乎没有摩擦）会一直悠悠地转，甚至在过零后反向漂。
               * 加一个恒定的库仑分量，保证最后几十 rpm 也能干脆停住。 */
              float brake = RC_WHEEL_STOP_KP * wheel_abs(rpm) + RC_WHEEL_STOP_KP0;
              a = (rpm > 0.0f) ? -brake : brake;
              a = wheel_clamp(a, -RC_WHEEL_STOP_A, RC_WHEEL_STOP_A);
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

}

/* ------------------------------------------------------------------ */
/* 阶段 3：yaw（限位/超速兜底/斜坡）→ volt                             */
/* ------------------------------------------------------------------ */
static void RC_Yaw_Step(uint32_t now, uint8_t online, uint8_t spin_mode, float *out_volt)
{
    /* 6) 云台 GM6020（yaw）：限位 + 超速兜底 + 斜坡 + 安全开关
     *    小陀螺走专用世界航向环，其余模式走原来的手控/跟随环，两条路互不干扰 */
    float gim  = online ? RC_Norm(RC_CH_GIMBAL) : 0.0f;
    float v = (spin_mode != 0u) ? RC_Yaw_Spin_Control(online, gim)
                                   : RC_Yaw_Control(online, gim);

    /* ① 反馈超时 / 遥控离线 → 断电（没有反馈就等于没有限位）
     *    ★每次断电都留下“为什么”：原因码 + 历史最大反馈年龄 + 历史最大转速，
     *      不再靠 Watch 猜（Watch 在故障瞬间的读数不可信，教训见 2026-11-17 排查记录） */
    uint8_t yaw_kill     = 0u;
    uint8_t kill_reason  = 0u;

    {
        uint32_t fb_age  = now - gim6020.last_tick;
        float    rpm_abs = (float)gim6020.rpm;

        if (rpm_abs < 0.0f) rpm_abs = -rpm_abs;
        if (fb_age  > RC_YawFbAgeMax) RC_YawFbAgeMax = fb_age;
        if (rpm_abs > RC_YawSpdMax)  RC_YawSpdMax  = rpm_abs;

        if (online == 0u)                        { yaw_kill = 1u; kill_reason = 1u; }
        if (fb_age > GM_FEEDBACK_TIMEOUT_MS)      { yaw_kill = 1u; kill_reason = 2u; }

        /* 断流集数：故障是间歇性的，Ozone 不一定守得住现场；
         * 事后读 RC_YawOutageCnt 就知道断过几次（配合 RC_YawFbAgeMax 看最长一次多久） */
        {
            static uint16_t s_out_ms = 0u;

            if (fb_age > GM_FEEDBACK_TIMEOUT_MS) {
                if (s_out_ms == 0u) RC_YawOutageCnt++;
                if (s_out_ms < 60000u) s_out_ms++;
            } else {
                s_out_ms = 0u;
            }
        }

        /* ② 超速兜底（带去抖）：
         *    旧版单帧就断：0x205 里混进一帧别的节点的帧（|rpm| 几千）→ 每拍断电 →
         *    电压钉 0（“枪口无阻力”）+ s_yaw_volt_prev 清零 → 斜坡从零重爬 → 整车抽搐。
         *    现在：假帧在 DJ_Motor.c 已整帧丢弃（|rpm|>600）；这里再要求连续
         *    GM_RPM_KICK_MS(10ms) 都超限才认“真超速”，安全性不降。 */
        static uint16_t s_rpm_over = 0u;

        if (rpm_abs > GM_RPM_LIMIT) {
            if (s_rpm_over < 60000u) s_rpm_over++;
        } else {
            s_rpm_over = 0u;
        }
        if (s_rpm_over >= GM_RPM_KICK_MS)        { yaw_kill = 1u; kill_reason = 3u; }
    }

    RC_YawKill = (yaw_kill != 0u) ? kill_reason : 0u;
    if (yaw_kill != 0u) RC_YawKillCnt++;

    /* ④ 斜坡：每拍最多变 GM_VOLT_SLEW，避免扭矩冲击
     *    故障时不能走斜坡——否则上一拍 18000，这一拍只是 17800，
     *    要 90 ms 才掉到 0，等于故障期间还在给扭矩。直接把斜坡状态也清掉。 */
    if (s_yaw_slew_kick != 0u) {
        s_yaw_slew_kick = 0u;
        s_yaw_volt_prev = 0.0f;       /* 交接后从 0 重新爬，不吃上一模式的旧斜坡 */
    }
    if (yaw_kill != 0u) {
        v      = 0.0f;
        s_yaw_volt_prev = 0.0f;       /* 恢复时从 0 重新往上爬 */
    } else {
        if (v > s_yaw_volt_prev + GM_VOLT_SLEW) v = s_yaw_volt_prev + GM_VOLT_SLEW;
        if (v < s_yaw_volt_prev - GM_VOLT_SLEW) v = s_yaw_volt_prev - GM_VOLT_SLEW;
    }
    RC_YawSpinOut = v;       /* 实发电压（斜坡之后），小陀螺环的积分回算用它 */

    *out_volt = v;
}

/* ------------------------------------------------------------------ */
/* 阶段 4：摩擦轮 → 电流指令（含松杆主动刹车）                          */
/* ------------------------------------------------------------------ */
static void RC_Fric_Step(uint8_t online, int16_t *out_l, int16_t *out_r)
{
    /* ⑧ 摩擦轮：左拨杆上 → 3508 ID2/ID3 反向同电流 */
#if (FRIC_ENABLE_USE_LEFT)
    uint8_t fric_sw = (RC_Data.s_left  == FRIC_ENABLE_SW);
#else
    uint8_t fric_sw = (RC_Data.s_right == FRIC_ENABLE_SW);
#endif

    float fric_tgt = (online && fric_sw) ? FRIC_CURRENT_A : 0.0f;

    if (dj_motor[1].temp > FRIC_TEMP_LIMIT || dj_motor[2].temp > FRIC_TEMP_LIMIT)
        fric_tgt = 0.0f;
    if ((BSP_Now() - dj_motor[1].last_tick) > FRIC_TIMEOUT_MS ||
        (BSP_Now() - dj_motor[2].last_tick) > FRIC_TIMEOUT_MS)
        fric_tgt = 0.0f;

    if      (fric_tgt > s_fric_a + FRIC_RAMP_A_PER_MS) s_fric_a += FRIC_RAMP_A_PER_MS;
    else if (fric_tgt < s_fric_a - FRIC_RAMP_A_PER_MS) s_fric_a -= FRIC_RAMP_A_PER_MS;
    else                                             s_fric_a  = fric_tgt;

    int16_t fric_l;
    int16_t fric_r;

    if (fric_tgt > 0.0001f)
    {
        /* 正常出力：两轮反向，把弹丸往外挤 */
        fric_l = (int16_t)(FRIC_L_SIGN * s_fric_a / 20.0f * 16384.0f);
        fric_r = (int16_t)(FRIC_R_SIGN * s_fric_a / 20.0f * 16384.0f);
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

    *out_l = fric_l;
    *out_r = fric_r;
}
