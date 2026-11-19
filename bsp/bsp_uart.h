#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"

/* DBUS 一帧 18 字节。这个数只跟协议有关，放在 bsp 是为了让接收逻辑
 * 自成一体（模块层只需知道“来了一帧 18 字节的数据”）。 */
#define BSP_UART_DBUS_FRAME_LEN   18u

/* 收满一帧时回调：data 指向 bsp 内部 DMA 缓冲，长度固定 18，回调里要用就当场拷走。 */
typedef void (*BSP_UART_RxHandler)(const uint8_t *data, uint16_t len);

/* 注册解析函数（在启动之前调用；bsp 不认识遥控器，只负责把帧送上来） */
void BSP_UART_DBusRegisterRx(BSP_UART_RxHandler handler);

/* 注册错误钩子（模块层用它给 RC_ErrCnt 计数，bsp 不碰上层变量） */
void BSP_UART_DBusRegisterError(void (*cb)(void));

/* 启动/重启一次 UART5(IDLE)+DMA 接收；错误回调内部会自动重启 */
void BSP_UART_DBusStart(void);

#endif
