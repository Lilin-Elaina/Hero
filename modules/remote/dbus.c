/**
 * @file    dbus.c
 * @brief   DT7 + DR16 遥控器 DBUS 帧解析（收帧在 bsp_uart）
 * @note    硬件：达妙 DM-MC-Board02 SBUS 口（官方例程走 UART5 / RX=PD2 AF8）
 *          协议：100000bps, 8数据位+偶校验, 2停止位, 每帧18字节
 *          2026-11-19 分层整理：内容自 remote/rc_dbus.c 原样搬迁，逻辑未改；
 *          UART 收帧细节已下沉 bsp_uart（P2c），本文件只管协议解析。
 *          2026-11-20 清理：合并重复的文件头与 include，去掉搬迁时带过来的无关头。
 */
#include "dbus.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include <string.h>

RC_TypeDef RC_Data = {0};

/* 启动串口接收 */
void RC_StartReceive(void)
{
    BSP_UART_DBusStart();
}

/* bsp 收到一帧 18 字节后回调到这里 */
static void DBUS_OnRx(const uint8_t *data, uint16_t len)
{
    if (len >= DBUS_FRAME_LEN)
    {
        RC_Decode(data);
    }
}

/* UART 错误计数：bsp 出错时回调，计数变量仍在本工程 diag 区（application/robot.c 定义） */
static void DBUS_OnError(void)
{
    RC_ErrCnt++;
}

/*清除摇杆数据，在线状态，确定拨杆，开启接收*/
void RC_Init(void)
{
    memset(&RC_Data, 0, sizeof(RC_Data));
    RC_Data.online = 0;
    RC_Data.sw_swap = RC_SW_SWAP;
    BSP_UART_DBusRegisterRx(DBUS_OnRx);
    BSP_UART_DBusRegisterError(DBUS_OnError);
    RC_StartReceive();
}

/* ------------------------------------------------------------------ */
/* 18 字节帧完整解析（与大疆官方 sbus_to_rc() 逐位一致）                */
/*   byte0~5  : 4 路摇杆×11bit + 2 路拨杆×2bit                         */
/*   byte6~13 : 鼠标 x/y/z + 左右键                                    */
/*   byte14~15: 键盘按键位图                                           */
/*   byte16~17: 左拨轮(11bit)                                          */
/* 左拨杆在 b[5] 的哪个 2bit 位域，由 dbus.h 的 RC_SW_SWAP 编译期固定：
 *   0 = bit4~5（官方 s[0]），1 = bit6~7（官方 s[1]）
 * 不再上电自动判定——换遥控器后若左拨杆失灵，只改那一个宏。 */

void RC_Decode(const uint8_t *b)
{
    /* ---- 4 路摇杆 × 11bit，LSB 先 / 小端顺序打包 ---- */
    RC_Data.ch[0] = (uint16_t)(b[0]            | ((uint16_t)b[1] << 8))                 & 0x07FF;
    RC_Data.ch[1] = (uint16_t)((b[1] >> 3)     | ((uint16_t)b[2] << 5))                 & 0x07FF;
    RC_Data.ch[2] = (uint16_t)((b[2] >> 6)     | ((uint16_t)b[3] << 2) | ((uint16_t)b[4] << 10)) & 0x07FF;
    RC_Data.ch[3] = (uint16_t)((b[4] >> 1)     | ((uint16_t)b[5] << 7))                 & 0x07FF;

    /* ---- 2 路拨杆，各 2bit，放在 b[5] 的高 4 位 ----
     *   sw_lo2 = (b[5]>>4)&0x03       = 官方 s[0]（bit4~5）
     *   sw_hi2 = ((b[5]>>4)&0x0C)>>2  = 官方 s[1]（bit6~7）
     * 哪个是物理左拨杆由 dbus.h 的 RC_SW_SWAP 编译期指定，位运算不用动。*/
    RC_Data.sw_lo2 = RC_SW_BITS_LO(b[5]);
    RC_Data.sw_hi2 = RC_SW_BITS_HI(b[5]);

    if (RC_Data.sw_swap)
    {
        RC_Data.S_Left  = RC_Data.sw_hi2;
        RC_Data.S_Right = RC_Data.sw_lo2;
    }
    else
    {
        RC_Data.S_Left  = RC_Data.sw_lo2;
        RC_Data.S_Right = RC_Data.sw_hi2;
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

    {
        uint32_t now = BSP_Now();

        /* 两帧间隔统计：RC_GapMaxMs > 100 就是一次实打实的“准失联”，
         * 这比 online 标志可信——online 只看最后一帧距今，间隔大不大它不会记。 */
        if ((RC_Data.last_tick != 0u) && ((now - RC_Data.last_tick) > RC_GapMaxMs))
        {
            RC_GapMaxMs = now - RC_Data.last_tick;
        }
        RC_FrameCnt++;
        RC_Data.last_tick = now;
    }
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
