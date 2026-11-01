#ifndef __RC_DBUS_H
#define __RC_DBUS_H     

#include "main.h"

/* DT7+DR16 进行解码文件，通过发送无线帧，到DR16，DR16通过DBUS协议发送反串口电平，经过妙板硬件反电平，输出正常串口电平 进行解码。*/
#define DBUS_FRAME_LEN   18     /*大疆DBUS协议18字节数据*/
#define RC_RX_INVERT     0      /* 0=达妙板上有硬件反相器(默认,正确); 1=仅用于绕开板载反相器时 */

#define RC_CH_MID        1024    /* 摇杆通道中值     */
#define RC_CH_MIN        364     /* 摇杆通道最小值     */
#define RC_CH_MAX        1684    /* 摇杆通道最大值     */
#define RC_DEADBAND      20      /* 中位死区     */
#define RC_OFFLINE_MS    100     /*超过100ms认为失联（失联时间） */

/* ---- 控制参数（按需要改）------------------------------------------ */
#define RC_MAX_CURRENT_A  5.0f   /* 单电机最大电流(A)，建议先 1.0~2.0 试 */
#define RC_ABS_CURRENT_A  6.0f   /* 硬性绝对上限(A)，任何情况都不超过     */
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

/* 电机方向：电机装在底盘左侧/右侧时，为避免整车“转圈”，其中一侧通常要取反。
 * 判断方法见 _rc_dbus_control_logic.md 第 5 节“首次通电自检”。
 * 四轮对应关系（与 mecanum.h 的 MEC_WHEEL_LF/LB/RB/RF 一致）：
 *   ID1=左前  ID2=右前  ID3=右后  ID4=左后
 * ★ 整车“前进变后退”就四个一起翻；“原地打转”就把某一侧两个一起翻。 */
#define M1_SIGN          (+1.0f) /* 左前(ID1)转向取反就改成 -1.0f */
#define M2_SIGN          (+1.0f) /* 左后(ID2) */
#define M3_SIGN          (+1.0f) /* 右后(ID3) */
#define M4_SIGN          (+1.0f) /* 右前(ID4) */

/* ---- 摇杆通道映射（依据官方 sbus_to_rc 解码，见说明文档第 1.4 节）----
 * 官方 18 字节帧解析出的 4 个摇杆：
 *   ch[0] = 右摇杆水平  ch[1] = 右摇杆垂直  ch[2] = 左摇杆水平  ch[3] = 左摇杆垂直
 * 出厂默认“左手油门”，所以：
 *   前进/后退 = ch[3]（左摇杆上下）
 *   自转      = ch[2]（左摇杆左右），也可换成 ch[0]（右摇杆左右）
 *   横移      = ch[0]（右摇杆左右；2 个轮子时无意义，补到 4 轮才启用）
 * ★ 2026-10-23 修正：原来下面两行和这段注释正好接反（推前进变原地转） */
#define RC_CH_FWD        RC_Data.ch[2]   /* 左摇杆上下 = 前进/后退 */
#define RC_CH_TURN       RC_Data.ch[3]   /* 左摇杆左右 = 自转 */
#define RC_CH_STRAFE     RC_Data.ch[0]   /* 右摇杆左右 = 横移（与云台 yaw 共用，见下） */
/* ---- 云台 GM6020（只控 ID1）---- */
#define RC_CH_GIMBAL     RC_Data.ch[0]   /* 右摇杆水平；没反应就改 ch[1] */
#define GM6020_VOLT_MAX  15000.0f         /* 电压上限（约 8V） */
#define GM6020_SIGN      (-1.0f)         /* 方向反了就 -1.0f */

/* ---- 软件限位（原始角度 0~8191，先填占位值，实测后改）---- */
#define GM_ANGLE_MIN     5000           /* 允许的最小角度 */
#define GM_ANGLE_MAX     7000         /* 允许的最大角度 */
#define GM_SOFT_ZONE     300             /* 提前 300 counts（约 13°）开始减速 */
#define GM_FEEDBACK_TIMEOUT_MS  100      /* 超过 100ms 没反馈 → 输出 0 */
#define GM_RPM_LIMIT     400.0f          /* 超速兜底：|rpm| 超过这个值就断电压 */
#define GM_VOLT_SLEW     200.0f          /* 斜坡：每 1ms 电压最多变化这么多 */
/* ---- 云台 pitch（6020 ID2）---- */
#define RC_CH_GIMBAL_PITCH    RC_Data.ch[1]    /* 右摇杆垂直 */
#define GM_PITCH_SIGN         (+1.0f)          /* 方向反了改 -1.0f */
#define GM_PITCH_VOLT_MAX     12000.0f         /* 电压上限 */
#define GM_PITCH_ANGLE_MIN    0                /* ★ 先不限位，量完再收紧 */
#define GM_PITCH_ANGLE_MAX    8191             /* ★ 先不限位，量完再收紧 */
#define GM_PITCH_SOFT_ZONE    200              /* 提前约 8.8° 减速 */

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
 *                   4 轮时 [0]=左前 [1]=左后 [2]=右后 [3]=右前 */
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
