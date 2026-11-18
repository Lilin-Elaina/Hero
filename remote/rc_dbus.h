#ifndef __RC_DBUS_H
#define __RC_DBUS_H     
/*控制各电机，解码，的宏定义以及函数声明*/
#include "main.h"

/* DT7+DR16 进行解码文件，通过发送无线帧，到DR16，DR16通过DBUS协议发送反串口电平，经过妙板硬件反电平，输出正常串口电平 进行解码。*/
#define DBUS_FRAME_LEN   18     /*大疆DBUS协议18字节数据*/
#define RC_RX_INVERT     0      /* 0=达妙板上有硬件反相器(默认,正确); 1=仅用于绕开板载反相器时 */

#define RC_CH_MID        1024    /* 摇杆通道中值     */
#define RC_CH_MIN        364     /* 摇杆通道最小值     */
#define RC_CH_MAX        1684    /* 摇杆通道最大值     */
#define RC_DEADBAND      20      /* 摇杆中位死区     */
#define RC_OFFLINE_MS    100     /*超过100ms认为失联（失联时间） */

/* ---- 控制参数（按需要改）------------------------------------------ */
#define RC_MAX_CURRENT_A  7.0f   /* 单电机最大电流(A)，建议先 1.0~2.0 试 */
#define RC_ABS_CURRENT_A  8.0f   /* 硬性绝对上限(A)，任何情况都不超过     */
/* 拨杆位置编码（官方解码：1=上 2=下 3=中） */
#define RC_SW_MID          3
#define RC_SW_UP           1
#define RC_SW_DOWN         2

/* ---- 拨杆位域位置（与具体帧版本无关，纯位运算）----
 * b[5] 的 bit6~7 与 bit4~5 各放一个 2bit 拨杆。 */
#define RC_SW_BITS_HI(b5)   ((uint8_t)(((b5) >> 4) & 0x0C) >> 2)   /* bit6~7 */
#define RC_SW_BITS_LO(b5)   ((uint8_t)(((b5) >> 4) & 0x03))        /* bit4~5 */

#define RC_ENABLE_SW_POS   RC_SW_MID  /* 使能拨杆要停在这个位置才允许输出电流；
                                       * 改 0 = 不设使能开关（上电即受控，谨慎） */
#define RC_ENABLE_USE_LEFT 0     /* 用哪个物理拨杆做使能：0=右拨杆(默认) 1=左拨杆
                                  * 若调试时发现左右命名反了，把 RC_Data.sw_swap 置 1 即可 */
#define RC_ENABLE_DELAY_MS 100   /* 使能条件需连续保持这么久才真正放行（防抖软启动） */
#define RC_FAILSAFE_MS    500    /* 失联后开始强制刹停的时间（配合斜坡，不是硬断） */

/*===================================================
 *   底盘四轮速度闭环
 *=================================================== */

#define RC_WHEEL_ENABLE_PID    1        /*1 启动pid*/
#define RC_WHEEL_SPD_DT_S      0.001f   /*1ms调用一次*/

#define RC_WHEEL_RPM_MAX       8000.0f   /*rpm*/
#define RC_WHEEL_SPD_KP        0.002f      /*速度环比例系数*/
#define RC_WHEEL_SPD_KI        0.002f     /*速度环积分系数*/
#define RC_WHEEL_FF_A_PER_RPM  0.0008f    /*前馈系数：按 7758rpm≈7A 估的起点，实测再校准*/
#define RC_WHEEL_I_MAX         1.5f       /*积分只做静差微调，封顶1.5A*/

#define RC_WHEEL_STOP_KP       0.002f     /*回中制动系数A/rpm：输出=-STOP_KP*rpm，恒逆实测转速*/
#define RC_WHEEL_STOP_A        3.5f       /*回中制动限流A*/
#define RC_WHEEL_STOP_QUIET_RPM 50.0f     /*轮速低于此值撤掉制动电流，杜绝静止残余顶力*/

#define RC_WHEEL_FEEDBACK_MS   100u        /*超过这个时间没反馈就认为失联，输出 0 */
#define RC_WHEEL_TEMP_LIMIT    75u        /*温控值，超过这个温度就不输出电流*/
#define RC_WHEEL_FAULT_ISOLATE 1      /*1=只停故障轮，其余三轮照跑；0=整车断电*/

/* ---- 底盘跟随（右拨杆中位）：把车头追向枪口，α→0 ----
 * 用云台编码器算相对角，零漂、不依赖陀螺；速度环仍在下面闭环 */
#define RC_FOLLOW_KP       0.015f   /*自转指令 / 度：α=10° 给 0.20
                                     *尾段慢（最后几度拖 2 s）就加这个：0.020→0.026。
                                     *别一次跳太大，0.030 以上在旧版律下摆过*/
#define RC_FOLLOW_KICK     0.035f   /*起步量：边界处为 0，随 |e| 线性长到满值，否则小角度推不动车*/
#define RC_FOLLOW_DEAD     2.5f     /*死区，度：±2.5° 内不纠，免得到位后左右蹭*/
#define RC_FOLLOW_KD       0.0008f  /*阻尼：自转指令 per(deg/s)，α̇ 直接用 6·rpm（编码器微分，不过陀螺）。
                                     *纯 P 时冲过头再回来就是"摆"，这一项负责提前收油*/
#define RC_FOLLOW_MAX      0.70f    /*自转指令上限（占满量程比例）：这是"劲儿/速度"总闸，
                                     *嫌追得太冲就 0.70→0.65；尾段利不利索由 KP 管，别混*/
#define RC_FOLLOW_SIGN     1.0f     /*方向：实车若越追越偏就改成 -1.0f*/

/* ---- 小陀螺（右拨杆上位）：底盘基础自转 ---- */
#define GM_SPIN_BASE       0.45f    /*基础自转指令（占满量程）：0.45 约半速*/
#define RC_SPIN_TRIM       0.50f    /*转轮摇杆对自转的微调权重*/

/* ---- 枪口系换系 ---- */
#define GM_GUN_ANGLE_SIGN  (-1.0f)  /*2026-11-13 表二实测：α=±90 两组全反 → 用 -α 旋转*/

/* ---- 云台稳向（底盘转动补偿）---- */
#define GM_YAW_STAB_SIGN   1.0f         /*方向：逆时针推车，枪口若跟着转就改 -1.0f*/
#define GM_YAW_STAB_DEAD     2.0f       /*底盘角速度低于此值不补（deg/s）：静止时完全不积分，无零偏漂移
                                         *★注意：跟随模式必须开着它。关掉后底盘转、关节角不变，
                                         *  跟随环永远看不到误差收敛，就会一直自转下去。*/
#define GM_YAW_COUNT_PER_DEG 22.755556f /*8192/360：度 → 编码器 count*/

/* ---- 世界航向保持（松杆后锁住枪口朝向）----
 * 松杆（d_ref==0）时按实测枪口角速度反向补偿目标角，等效把世界方位钉住。
 * K 越大残差越小，但太大自激：残差 = ω/(1+K)，K=1→50% K=2→33% K=4→20% K=10→9%
 * ★2026-11-13 22:00 诊断期间置 0（关闭稳向），先把 yaw 速度环单独调稳。 */
#define GM_YAW_HOLD_K      2.0f         /*世界稳向增益：残差 ω/(1+K)，K=2→33%
                                         *★跟随/小陀螺都必须开：关掉后关节被位置环钉在车体上，
                                         *  车转 gim6020.angle 不变 → α 永远不变 → 跟随环永远追不到 0，会一直自转*/
#define GM_YAW_HOLD_DEAD   1.5f         /*|ω| 低于此值(deg/s)不补*/
#define GM_YAW_HOLD_SIGN   1.0f         /*★稳向方向：若小陀螺时枪口越稳越跑（转得比底盘还快）就改 -1.0f*/
#define GM_YAW_HOLD_KI     2.0f         /* 稳向积分（1/s）：把比例残差压到 0，太大调小到 1.0 */
#define GM_YAW_HOLD_RATE_MAX 300.0f     /*稳向补偿角速度上限 deg/s（抵消 220°/s 的自转够用，留点余量）*/

/* ---- 小陀螺专用 yaw 环（独立于跟随，2026-11-14）----
 * 结构：世界航向 P → 关节转速前馈（实测底盘角速度）→ 复用 SPD_KP 速度环。
 * 与 GM_YAW_HOLD_* 那套"挪位置目标"的稳向并行存在，靠模式选择二选一。 */
#define SPIN_YAW_USE_JOINT_POS 1        /* 1=把世界目标换算成关节目标，用关节位置环（刚，抗手推）；
                                          * 0=旧的"世界速率环"（软，先被推走再慢慢追） */
#define SPIN_YAW_AIM_RATE   220.0f      /* 摇杆满杆的世界目标角速度 deg/s */
#define SPIN_YAW_KV         94.0f       /* 反电势前馈：电压单位/rpm（≈30000 单位 / 320 rpm）。
                                          * 转 200°/s = 33 rpm 要 3133 单位，全靠 P 顶就要
                                          * 60°/s 的稳态误差；有了它误差≈0 */
#define SPIN_YAW_POS_KP     1.2f        /* 航向误差(deg) → 角速度(deg/s)，1/s。
                                          * 低频来回晃就降到 1.0，跟得慢就升到 2.5 */
#define SPIN_YAW_SPD_KP     300.0f      /* 世界角速度环 P：V per rpm（1800 V per deg/s）。
                                          * 与跟随共用口径但独立可调；高频抖就降到 200 */
#define SPIN_YAW_KI         120.0f      /* 只补摩擦/负载偏差：有 KV 前馈后不需要大积分，
                                          * 大了反而在稳态附近来回蹭 */
#define SPIN_YAW_I_MAX      4000.0f     /* 积分限幅（电压单位，≈42 rpm 的稳态电压） */
#define SPIN_YAW_KAW        50.0f       /* 抗饱和回算 1/s：用"实发电压−请求电压"把积分拉回。
                                          * 200/拍斜坡拉起 18000 要 90 ms，这期间只按"未超限"
                                          * 判断就会一直积分，追上后必过冲。公式必须带 dt */
#define SPIN_YAW_RPM_MAX    250.0f      /* 关节转速上限 rpm（同 GM_YAW_RPM_MAX） */
#define SPIN_YAW_RATE_MAX   600.0f      /* 修正角速度上限 deg/s：防止 180° 折返错误把关节打满 */


extern  volatile uint8_t  RC_WheelFault;   /* 1=在线，0=失联 */
void    RC_Wheel_Reset(void);

/* 每个轮子的目标转速 rpm——专供 Ozone 观察，控制逻辑不读它。
 * 下标：0=左前 1=右前 2=右后 3=左后（= MECANUM_WHEEL_NUM）
 * 已乘 MOTOR_POL 极性，单位 rpm，范围 ±RC_WHEEL_RPM_MAX(8000) */
extern float RC_WheelRef[4];
/* yaw 闭环调试镜像（Ozone 直接写变量名） */
extern volatile float RC_YawRef;      /* 目标角 counts */
extern volatile float RC_YawErr;      /* 角度误差 counts */
extern volatile float RC_YawRpmDes;   /* 目标转速 rpm */
extern volatile float RC_YawVolt;     /* 斜坡之前的电压指令 */
extern volatile float RC_YawVoltOut;  /* 斜坡之后、真正发出去的电压 */
extern volatile float RC_YawI;        /* 速度环积分 */
extern volatile float RC_HoldWz;      /* 稳向看到的枪口世界角速度 deg/s */
extern volatile float RC_HoldRate;    /* 稳向补偿角速度 deg/s */
/* 小陀螺环调试镜像 */
extern volatile float RC_SpinYaw;     /* 世界航向实测 deg（多圈连续） */
extern volatile float RC_SpinWch;     /* 底盘角速度实测 deg/s */
extern volatile float RC_SpinErr;     /* 航向误差 deg（±180 内） */
void RC_Yaw_Handover(void);           /* 模式切换时交接两个 yaw 控制器的状态 */

/* ---- 掉线 / yaw 断电取证镜像（2026-11-17 排查“yaw 无阻力”加）----
 * Ozone 直接看；“Max/Age/MaxMs”类历史最大值，调试器里写 0 即可复位重新统计。
 * 断电原因一次读清楚，不再靠猜：
 *   RC_YawKill=1 遥控离线        → 查 RC_FrameCnt/RC_GapMaxMs
 *   RC_YawKill=2 yaw 反馈超时    → 查 RC_YawFbAgeMax（>100 即发生过）
 *   RC_YawKill=3 超速(去抖后)    → 查 RC_YawSpdMax、can_rx_205_bad、RC_Can2TEC/REC
 */
extern volatile uint32_t RC_FrameCnt;     /* DBUS 解析出的帧数（正常约 71 帧/s） */
extern volatile uint32_t RC_GapMaxMs;     /* DBUS 相邻两帧最大间隔 ms（正常 14~15） */
extern volatile uint32_t RC_ErrCnt;       /* UART5 错误回调次数（ORE/FE/NE/PE 每条 +1） */
extern volatile uint32_t RC_YawFbAgeMax;  /* yaw 反馈最大年龄 ms（>100 → fresh 兜底断电） */
extern volatile float    RC_YawSpdMax;    /* yaw 接受过的帧里最大 |rpm|（6020 物理极限 ~320） */
extern volatile uint8_t  RC_YawKill;      /* 本拍断电原因：0=正常 1=离线 2=反馈超时 3=超速 */
extern volatile uint32_t RC_YawKillCnt;   /* 断电累计拍数（1kHz 调用，2000 = 累计断过 2 秒） */
extern volatile uint32_t RC_Can2TEC;      /* CAN2 发送错误计数 TEC（>127 = 错误被动） */
extern volatile uint32_t RC_Can2REC;      /* CAN2 接收错误计数 REC（>127 = 错误被动） */
extern volatile uint32_t RC_Can2TECMax;  /* TEC 历史最大值 */
extern volatile uint32_t RC_Can2RECMax;   /* REC 历史最大值 */
extern volatile uint32_t RC_YawOutageCnt; /* yaw 反馈断流次数（每断一次 +1，写 0 复位） */
extern volatile uint32_t RC_Can2PSR;      /* CAN2 PSR 寄存器：bits[2:0]=LEC 最后错误码，bit7=BO，bit9=ACT */
extern volatile uint32_t RC_Can2CCCR;     /* CAN2 CCCR 寄存器：bit0=INIT（1=控制器停着） */
extern volatile uint32_t RC_Can2BusOffCnt;/* CAN2 Bus-Off 中断次数（累计，写 0 复位） */
extern volatile uint32_t RC_Can2RestartCnt;/* CAN2 RX 看门狗重启次数（连续 20ms 无帧触发，写 0 复位） */
extern volatile uint32_t RC_BuildTag;     /* 固件版本指纹：应为 0x11172358，不符 = Ozone 挂了旧 ELF */
/* 0x205 假帧证据在 DJ_Motor.h：can_rx_205_bad / can_rx_205_bad_rpm */

/*
+ * CAN2 底盘电机指令方向补偿。
+ * 乘在整轮输出上，同时作用于前进、横移和自转。
+ * +1：保持混控输出符号；-1：反转混控输出符号。
+ * 数值沿用本车已验证配置，按 CAN 电机 ID 排列。
+ */
#define MOTOR_POL_ID1    (+1.0f)   /* 左前 LF */
#define MOTOR_POL_ID2    (-1.0f)   /* 右前 RF */
#define MOTOR_POL_ID3    (-1.0f)   /* 右后 RB */
#define MOTOR_POL_ID4    (+1.0f)   /* 左后 LB */

/*通道映射*/
#define RC_CH_FWD        RC_Data .ch[3]  /* 左摇杆上下 = 前进/后退 */
#define RC_CH_TURN       RC_Data .wheel  /* 左摇杆左右 = 自转 */
#define RC_CH_STRAFE     RC_Data.ch[2]   /* 右摇杆左右 = 横移 */
/* ---- 云台 GM6020（只控 ID1）---- */
#define RC_CH_GIMBAL     RC_Data.ch[0]   /* 右摇杆水平； */
#define GM6020_VOLT_MAX  15000.0f         /* 电压上限（约 8V） */
#define GM6020_SIGN      (-1.0f)         /* 方向反了就 -1.0f */

/*云台yaw */
#define GM_FEEDBACK_TIMEOUT_MS  100      /* 超过 100ms 没反馈 → 输出 0 */
#define GM_RPM_LIMIT     500.0f          /* 超速兜底：|rpm| 连续超此值才断电压。
                                          * 6020 物理极限 ~320rpm（空载），400 以内正常可达不了；
                                          * 真超速（控制律跑飞）持续 GM_RPM_KICK_MS 拍必被掐死。
                                          * 单帧毛刺（别的节点顶 0x205 名）由 DJ_Motor.c 的
                                          * |rpm|>600 整帧过滤 + 这里的去抖共同挡住，不再每拍断电 */
#define GM_RPM_KICK_MS   10u             /* 超速去抖：连续这么多个 1ms 拍都超才真正断电 */
#define GM_VOLT_SLEW     200.0f          /* 斜坡：每 1ms 电压最多变化这么多 */

/* ---- 云台 yaw 角度闭环（2026-11-13 实测标定后启用）----
 *  实测：逆时针（俯视）= gim6020.angle 增大 = gim6020.rpm 为正
 *        INS_Gyro[0] 在同一转动下为负 ⇒ ω_yaw = -INS_Gyro[0]
 *  信号流：摇杆 → 目标角 s_ref → 位置P → 目标转速 → 速度PI → 电压
 */
#define GM_YAW_POS_DIR      1.0f         /* 角度误差→转速：正 rpm 使 angle 增大 */
#define GM_YAW_STICK_RATE   5000.0f      /* 满杆目标角速度 counts/s ≈ 220°/s */
#define GM_YAW_ANG_KP       0.12f        /* 位置 P：rpm / count（诊断后从 0 恢复）
                                          * 100 counts 误差 → 12 rpm 目标，够顶住静摩擦 */
#define GM_YAW_ANG_KD       0.20f        /* 位置 D：rpm / rpm（原 0.60 在 250 rpm 时算出 150 rpm，
                                          * 直接吃掉 RPM_MAX 的 60%，等于减速段一直满舵 → 冲过再回来）*/
#define GM_YAW_ANG_D_MAX    40.0f        /* D 项单独限幅：±40 rpm 封顶，防止大转速时 D 项饱和 */
#define GM_YAW_RPM_MAX      250.0f       /* 目标转速限幅 */
#define GM_YAW_SPD_KP       300.0f       /* 速度 P：电压 / rpm */
#define GM_YAW_SPD_KI       0.0f         /* 速度 I：★诊断配置置 0（原 80），测完再按需加回 */
#define GM_YAW_I_MAX        8000.0f      /* 积分限幅 */
#define GM_YAW_ARRIVE_CNT   30.0f        /* 到位判定：位置误差 counts（30≈1.3°） */
#define GM_YAW_ARRIVE_RPM   15.0f        /* 到位判定：转速 rpm，两条同时满足就清积分 */
#define GM_YAW_VOLT_MAX     18000.0f     /* 电压限幅 */

/* ---- 枪口系平移（阶段 3/4）----
 *  ALIGN = 云台正对车头时 gim6020.angle 的读数（2026-11-13 实测 = 100）
 *  α = wrap(angle - ALIGN)：α>0 表示枪口在车头左侧（逆时针）
 *  摇杆按枪口方向解释：v_body = R(α) · v_gun
 *        vx_b = cosα·vx_g - sinα·vy_g
 *        vy_b = sinα·vx_g + cosα·vy_g
 *  GM_GUN_ANGLE_ENABLE = 0 时退回纯车体系（摇杆方向 = 车头方向）
 */
#define GM_YAW_ALIGN_COUNT   100.0f
#define GM_GUN_ANGLE_ENABLE  1u

extern volatile float RC_GunAngle;       /* 调试镜像：当前 α（度，-180~+180） */
/* ==============================================================================
 * 云台pitch:位置P + 速度PI环 ->转矩电压
 * 
 * 信号流:摇杆——参考角s_ref——位置p（角度差——目标转速）
 *                -速度pi——0x1FF
 * 
 * 量纲（8192 counts/圈 = 360）
 *     1 count ~0.044°      1rpm  ~136.5 counts/s
 *6020 电压量程 ±30000，空载约 320rpm ⇒ 1 rpm ≈ 94 电压单位
 *    位置环 POS_KP = 1/(τ×136.5)，τ≈30ms → 0.25
 *    速度环 SPD_KP ≈ 94×α，α≈1 → 100
 * ================================================================================*/

#define RC_CH_GIMBAL_PITCH      RC_Data.ch[1]     /*右摇杆垂直*/
#define GM_PITCH_CALIBRATED     1               /*标定完改1使能*/
#define GM_PITCH_SIGN           (+1.0f)           /*位置方向*/
#define GM_PITCH_RPM_SIGN       (+1.0f)           /*转速方向*/
#define GM_PITCH_ANGLE_MIN      3450              /*机械限min*/
#define GM_PITCH_ANGLE_MAX      4790              /*机械限位max*/
#define GM_PITCH_ANGLE_MARGIN   40                /*留余count量*/

#define GM_PITCH_HOME_ANGLE     4100               /*回中值*/
#define GM_PITCH_HOME_RATE      500.0f             /*回中角速度counts/s*/
#define GM_PITCH_MANUAL_RATE    1800.0f            /*满杆角速度*/
#define GM_PITCH_HOME_TOL       8                  /*回中到位判定的余量counts值*/
#define GM_PITCH_ACCEL          12000.0f            /*加速度*/
#define GM_PITCH_DECEL          12000.0f           /*减速度*/




#define GM_PITCH_WAIT_MS         100               /*接管前“稳定+摇杆居中”要保持多久*/
#define GM_PITCH_STICK_DEAD      0.10f             /*摇杆视为居中的阈值*/
#define GM_PITCH_HOME_RPM_TOL    30.0f             /*回中判断停稳的转速*/
#define GM_PITCH_HOME_SETTLE_MS  200                /*回中到位需保持多久*/
#define GM_PITCH_HOME_TIMEOUT_MS 3000                /*回中超时 ——故障*/
#define GM_PITCH_POS_KP         0.17              /*位置p   rpm/counts*/                
#define GM_PITCH_RPM_MAX        200.0f             /*位置环输出上限 rpm  */
#define GM_PITCH_SPD_KP        105.0f             /*速度p    volt/rpm*/
#define GM_PITCH_SPD_KI        310.0f             /*速度i    volt/（rpm*s）*/
#define GM_PITCH_I_MAX          10000.0f            /*积分限幅（电压单位）
                                                    * 静止时 volt = s_i（速度环 P 项贡献为 0），
                                                    * 所以要 > 顶住重力所需的电压 */
#define GM_PITCH_GRAV_FF        2500.0f             /*重力前馈（电压码）：顶住云台自重的常量电压
                                                    * 标定法：FF=0 静止读 s_i 稳态值，原样（含符号）填进来 */
#define GM_PITCH_VOLT_MAX       18000.0f           /*总输出限幅*/
#define GM_PITCH_TEMP_LIMIT     65                 /*温控值*/
#define GM_PITCH_HOME_ENABLE    1 
#define GM_PITCH_CATCH_RPM      60.0f              

/* ---- 摩擦轮（3508 ID2 / ID3）---- */
#define FRIC_ENABLE_USE_LEFT  1                /* 1=左拨杆 0=右拨杆 */
#define FRIC_ENABLE_SW        RC_SW_UP         /* 拨到"上"=开 */
#define FRIC_CURRENT_A        1.0f             /* ★ 先 1A 试 */
#define FRIC_L_SIGN           (-1.0f)          /* ★ 原来往内转，两轮对调 */
#define FRIC_R_SIGN           (+1.0f)
#define FRIC_RAMP_A_PER_MS    0.02f            /* 1A 约 50ms 爬上去 */
#define FRIC_TEMP_LIMIT       60               /* 温度保护 ℃ */
#define FRIC_TIMEOUT_MS       5000             /* ★ 调试期放大，防误锁 */
#define FRIC_BRAKE_A          0.8f             /* 松杆主动刹车电流(A) */
#define FRIC_BRAKE_RPM        300              /* |rpm| 低于此值就不刹了 */
#define FRIC_USE_CAN1         1                /* 1=摩擦轮/pitch 走 CAN1，0=走 CAN2 */
/* ★ 安全开关
 *   1 = 只收不发：CAN 照常收发、gim6020 照常更新，但输出电压强制 0，
 *       电机完全不动。用来在“没有机械限位”的情况下先用手量出角度范围。
 *   0 = 正常出力（必须先把 GM_ANGLE_MIN / GM_ANGLE_MAX 改成实测值！） */
#define GM_SAFE_NO_OUTPUT   0
/* ==================================================================
 *  拨杆位域（“哪个 bit 是左拨杆”有版本分歧，务必按下面方法一次性量准）
 *  b[5] 的高 4 位放两个 2bit 拨杆：bit4~5 与 bit6~7。
 *  大疆官方 sbus_to_rc() 原文（注意 s[0] 取的是低位）：
 *        s[0] = (b[5]>>4) & 0x03        → bit4~5
 *        s[1] = ((b[5]>>4) & 0x0C) >> 2 → bit6~7
 *  习惯上 s[0] 叫左拨杆、s[1] 叫右拨杆，但不同出处/遥控器批次偶有对调。
 *  本工程默认(sw_swap=0)：s_left = bit4~5(=s[0])，s_right = bit6~7(=s[1])。
 *  ★ 一次性验证法（本工程 D-Cache 未开，调试器读到的就是真值）：
 *      1) 静止时看 RC_Data.sw_hi2 / sw_lo2，都应是 3(中位)；
 *      2) 拨 DT7 左边那个拨杆，看 sw_hi2 还是 sw_lo2 在 1↔3 跳；
 *      3) 若跳的是 sw_hi2(bit6~7)，说明你这台是“左拨杆在高 2 位”，
 *         把 RC_Data.sw_swap 置 1，s_left/s_right 会自动对上；
 *         若跳的是 sw_lo2(bit4~5)，保持 0（默认即正确）。
 *      sw_swap 一定好就固定，位运算本身不用动。
 * ================================================================== */

/* 鼠标数据（DR16 帧 byte6~13），与大疆官方 RC_ctrl_t.mouse 同源
 * 控制层暂不使用，但解出来以保证"整帧 18 字节全覆盖、解析完整" */
typedef struct
{
    int16_t x;            /* X 位移(int16) */
    int16_t y;            /* Y 位移(int16) */
    int16_t z;            /* 滚轮位移。官方未掩码；此处 &0x07FF 为防御性加固 */
    uint8_t press_l;      /* 左键按下=1 */
    uint8_t press_r;      /* 右键按下=1 */
} RC_Mouse_t;

typedef struct
{
    uint16_t ch[4];       /* 4 路摇杆：364~1684，中值1024 */
    uint8_t  s_left;      /* 物理左拨杆：1=上 2=下 3=中（受 sw_swap 影响） */
    uint8_t  s_right;     /* 物理右拨杆：1=上 2=下 3=中（受 sw_swap 影响） */
    uint8_t  sw_hi2;      /* 原始位域 b[5] bit6~7 = 官方 s[1] */
    uint8_t  sw_lo2;      /* 原始位域 b[5] bit4~5 = 官方 s[0] */
    uint8_t  sw_swap;     /* ★命名开关：0=默认(s_left取bit4~5=官方s[0]) 1=互换 */
    uint16_t wheel;       /* 左拨轮(11bit)。官方 wheel 未掩码；此处 &0x07FF 为防御性加固 */
    RC_Mouse_t mouse;     /* 鼠标 x/y/z + 左右键 */
    uint8_t  key_v;       /* 键盘按键位图(纵向) = b[14] */
    uint8_t  key_h;       /* 键盘按键位图(横向) = b[15] */
    /* ---- 拨杆位域自动判定：上电后 3s 内做一次即可，自动锁定 ----
     * 实测法：把 DT7 左拨杆停在“上/下”(非中位)，右拨杆保持中位，持续约 1 秒。
     *   这一秒里“稳定处于非中位”的那个位域 = 物理左拨杆 → 自动置 sw_swap。
     * 判定成功即永久锁定(sw_auto_done=1)，之后运行中拨动不会改写。      */
    uint8_t  sw_auto_done;     /* 1 = 已自动判定完成并锁定 */
    uint8_t  sw_auto_result;   /* 0 = 左拨杆在 bit4~5(sw_swap=0)；1 = 在 bit6~7(sw_swap=1) */
    uint8_t  sw_obs_last;      /* 内部：上一帧观测状态(0~3) */
    uint8_t  sw_obs_hold;      /* 内部：当前观测状态已连续保持的帧数(调试观察) */
    uint32_t sw_obs_start_ms;  /* 内部：当前观测状态首次出现的时刻 */
    uint32_t sw_first_frame_ms;/* 内部：上电后第一帧到达的时刻 */
    uint32_t last_tick;   /* 最后一次收到帧的 HAL_GetTick() */
    uint8_t  online;      /* 1=遥控器在线 */
} RC_TypeDef;

extern RC_TypeDef RC_Data;

/* 最近一次 RC_Control_Task 算出的各轮归一化目标 [-1,1]（调试观察用）
 * 下标见 
 *                   4 轮时 [0]=左前 [1]=右前 [2]=右后 [3]=左后 */
extern float RC_WheelOut[];

void  RC_Init(void);                 /* 启动 DBUS 接收（UART5 + DMA + IDLE） */
void  RC_StartReceive(void);         /* 重启一次 DMA 接收 */
void  RC_Decode(uint8_t *buf);       /* 18 字节帧解析 */
void  RC_SwitchAutoName(void);       /* 拨杆位域自动判定（在 RC_Decode 内被调用） */
uint8_t RC_IsOnline(uint32_t now_ms);/* 失联检测 */
float RC_Norm(uint16_t ch);          /* 归一化函数，摇杆 → -1.0~+1.0（含死区） */

/* 每个控制周期调用一次：解码后的摇杆 →3508 的电流指令 */
void RC_Control_Task(void);

#endif
