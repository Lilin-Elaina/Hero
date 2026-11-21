#ifndef __DBUS_H
#define __DBUS_H
/* DBUS 协议层：帧格式、摇杆/拨杆通道定义、解码结果。
 * 这一层属于 modules/remote：只描述"遥控器发来了什么"，不含任何控制参数。
 * 所有控制参数、增益、调试镜像都在 application/rc_dbus.h。 */
#include "main.h"

/* DT7+DR16 进行解码文件，通过发送无线帧，到DR16，DR16通过DBUS协议发送反串口电平，经过妙板硬件反电平，输出正常串口电平 进行解码。*/
#define DBUS_FRAME_LEN   18     /*大疆DBUS协议18字节数据*/
#define RC_RX_INVERT     0      /* 0=达妙板上有硬件反相器(默认,正确); 1=仅用于绕开板载反相器时 */

#define RC_CH_MID        1024    /* 摇杆通道中值     */
#define RC_CH_MIN        364     /* 摇杆通道最小值     */
#define RC_CH_MAX        1684    /* 摇杆通道最大值     */
#define RC_DEADBAND      20      /* 摇杆中位死区     */
#define RC_OFFLINE_MS    100     /*超过100ms认为失联（失联时间） */

/* 拨杆位置编码（官方解码：1=上 2=下 3=中） */
#define RC_SW_MID          3
#define RC_SW_UP           1
#define RC_SW_DOWN         2

/* ---- 拨杆位域位置（与具体帧版本无关，纯位运算）----
 * b[5] 的 bit6~7 与 bit4~5 各放一个 2bit 拨杆。 */
#define RC_SW_BITS_HI(b5)   ((uint8_t)(((b5) >> 4) & 0x0C) >> 2)   /* bit6~7 */
#define RC_SW_BITS_LO(b5)   ((uint8_t)(((b5) >> 4) & 0x03))        /* bit4~5 */

/* ★物理左拨杆在哪个位域（编译期固定，不再上电自动判定）
 *   0 = b[5] bit4~5（官方 s[0]）
 *   1 = b[5] bit6~7（官方 s[1]）
 * 换遥控器/换固件后若“左拨杆推动时模式没反应”，把这里改成另一个值即可。*/
#define RC_SW_SWAP         1

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
    uint8_t  S_Left;      /* 物理左拨杆：1=上 2=下 3=中（受 sw_swap 影响） */
    uint8_t  S_Right;     /* 物理右拨杆：1=上 2=下 3=中（受 sw_swap 影响） */
    uint8_t  sw_hi2;      /* 原始位域 b[5] bit6~7 = 官方 s[1] */
    uint8_t  sw_lo2;      /* 原始位域 b[5] bit4~5 = 官方 s[0] */
    uint8_t  sw_swap;     /* 固定值 = RC_SW_SWAP（编译期决定 S_Left 取哪个位域） */
    uint16_t wheel;       /* 左拨轮(11bit)。官方 wheel 未掩码；此处 &0x07FF 为防御性加固 */
    RC_Mouse_t mouse;     /* 鼠标 x/y/z + 左右键 */
    uint8_t  key_v;       /* 键盘按键位图(纵向) = b[14] */
    uint8_t  key_h;       /* 键盘按键位图(横向) = b[15] */
    uint32_t last_tick;   /* 最后一次收到帧的 BSP_Now() */
    uint8_t  online;      /* 1=遥控器在线 */
} RC_TypeDef;

extern RC_TypeDef RC_Data;

/* ---- 掉线取证镜像（定义在 application/robot.c）---- */
extern volatile uint32_t RC_FrameCnt;     /* DBUS 解析出的帧数（正常约 71 帧/s） */
extern volatile uint32_t RC_GapMaxMs;     /* DBUS 相邻两帧最大间隔 ms（正常 14~15） */
extern volatile uint32_t RC_ErrCnt;       /* UART5 错误回调次数（ORE/FE/NE/PE 每条 +1） */

void  RC_Init(void);                 /* 启动 DBUS 接收（UART5 + DMA + IDLE） */
void  RC_StartReceive(void);         /* 重启一次 DMA 接收 */
void  RC_Decode(const uint8_t *buf);       /* 18 字节帧解析（只读，不修改缓冲） */
uint8_t RC_IsOnline(uint32_t now_ms);/* 失联检测 */
float RC_Norm(uint16_t ch);          /* 归一化函数，摇杆 → -1.0~+1.0（含死区） */

#endif /* __DBUS_H */
