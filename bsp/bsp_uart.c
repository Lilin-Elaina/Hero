/**
 * @file    bsp_uart.c
 * @brief   UART5 + DMA + IDLE 接收（DBUS 遥控器）
 * @note    分层：HAL 串口/DMA 细节收在 bsp，模块层只通过
 *          BSP_UART_DBusRegisterRx() 拿到“一帧 18 字节”。
 *          2026-11-19 从 modules/remote/dbus.c 原样搬迁，接收时序未改。
 */
#include "main.h"
#include "usart.h"
#include "bsp_uart.h"

/* DMA 接收缓冲：2 帧容量
 *
 * ★ 必须放在 DMA 可访问的内存里：STM32H7 的 DTCM(0x20000000) 只有 Cortex-M7
 *   的 D-bus 能访问，DMA1/DMA2 往 DTCM 写会在第一个字节就报传输错误。
 *   本工程 .data/.bss 都在 DTCMRAM，所以这里显式指定 .dma_buffer 段，
 *   该段在 STM32H723xG_flash.ld 里被链接到 RAM_D1(0x24000000)。
 *   32 字节对齐是为了将来若打开 D-Cache 仍能安全做 cache 维护。 */
#if defined(__GNUC__)
static uint8_t dbus_buf[BSP_UART_DBUS_FRAME_LEN * 2]
    __attribute__((section(".dma_buffer"), aligned(32)));
#else
static uint8_t dbus_buf[BSP_UART_DBUS_FRAME_LEN * 2];
#endif

static BSP_UART_RxHandler S_Dbus_Rx   = 0;   /* 模块层注册的解析函数 */
static void (*S_Dbus_Err)(void)       = 0;   /* 模块层注册的错误计数钩子 */

void BSP_UART_DBusRegisterRx(BSP_UART_RxHandler handler)
{
    S_Dbus_Rx = handler;
}
//callback
/*该函数形参为指向无参无返回值的函数的指针*/
void BSP_UART_DBusRegisterError(void (*cb)(void))
{
    S_Dbus_Err = cb;
}

void BSP_UART_DBusStart(void)
{
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

        if ((Size >= BSP_UART_DBUS_FRAME_LEN) && (S_Dbus_Rx != 0))
        {
            /* 收到多帧时取最后一帧完整数据 */
            S_Dbus_Rx(&dbus_buf[Size - BSP_UART_DBUS_FRAME_LEN], BSP_UART_DBUS_FRAME_LEN);
        }

        BSP_UART_DBusStart();   /* 1帧收完，重新启动dma */
    }
}

/* ------------------------------------------------------------------ */
/* UART 错误回调（ORE/FE/NE/PE）                                        */
/* 重开uart                                                                      */
/* ------------------------------------------------------------------ */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        if (S_Dbus_Err != 0)
        {
            S_Dbus_Err();                 /* 模块层自己记数（RC_ErrCnt） */
        }

        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        BSP_UART_DBusStart();
    }
}
