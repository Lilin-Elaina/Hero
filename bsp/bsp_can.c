#include "main.h"
#include "bsp_can.h"
#include "fdcan.h"

/*
 * @ brief: 开启fdcan2，并启动fdcan2的总线关闭中断与fifo0接收中断（控制底盘电机与云台yaw）
 * @ note:过滤器的配置必须在启动fdcan2之前。 
 */
void FDCAN2_Start(void)
{ 
    FDCAN_FilterTypeDef sFilterConfig = {

    .FilterType = FDCAN_FILTER_MASK,
    .FilterIndex = 0,
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
    .IdType = FDCAN_STANDARD_ID,
    .FilterID1 = 0x000,
    .FilterID2 = 0x000,
    .IsCalibrationMsg = 0, // 是否检验消息
    .RxBufferIndex = 0, // 接收缓冲区编号
    };

    HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);//配置启动
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/*
 *@brief :配置fdcan1过滤器，开启fdcan1，启动fdcan1的总线关闭中断与接收消息中断
 */
void FDCAN1_Start(void)
{
    FDCAN_FilterTypeDef sFilterConfig = {

    .FilterType = FDCAN_FILTER_MASK,
    .FilterIndex = 0,
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
    .IdType = FDCAN_STANDARD_ID,
    .FilterID1 = 0x000,
    .FilterID2 = 0x000,
    .IsCalibrationMsg = 0,
    .RxBufferIndex = 0,
    };

    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* 
 *@brief:总线关闭处理，关闭总线再重开
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{  
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) == 0u)
    {
        return;
    }

    if (hfdcan->Instance == FDCAN2)
    {
        HAL_FDCAN_Stop(hfdcan);
        FDCAN2_Start();
    }
    else if (hfdcan->Instance == FDCAN1)
    {
        HAL_FDCAN_Stop(hfdcan);
        FDCAN1_Start();
    }
}

/* ------------------------------------------------------------------
 * 统一用此函数发送can报文
 * ------------------------------------------------------------------ */
HAL_StatusTypeDef BSP_CAN_Send(uint8_t bus, uint32_t id, const uint8_t *data)
{
    FDCAN_TxHeaderTypeDef hdr = {0};
    FDCAN_HandleTypeDef *h = (bus == 1u) ? &hfdcan1 : &hfdcan2;

    hdr.Identifier = id;
    hdr.IdType = FDCAN_STANDARD_ID;
    hdr.TxFrameType = FDCAN_DATA_FRAME;
    hdr.ErrorStateIndicator = DISABLE;
    hdr.BitRateSwitch = DISABLE;
    hdr.FDFormat = DISABLE;
    hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker = 0;
    hdr.DataLength = FDCAN_DLC_BYTES_8;

    return HAL_FDCAN_AddMessageToTxFifoQ(h, &hdr, (uint8_t *)data);
}

void BSP_CAN_Stop(uint8_t bus)
{
    HAL_FDCAN_Stop((bus == 1u) ? &hfdcan1 : &hfdcan2);
}

/* ------------------------------------------------------------------
 * 接收：唯一的 HAL 回调留在 bsp；取到帧后按总线交给注册者解析。
 * ------------------------------------------------------------------ */


static BSP_CAN_RxHandler S_Rx_Handler[3] = {0};   /* [1]=CAN1，[2]=CAN2 */

void BSP_CAN_RegisterRx(uint8_t bus, BSP_CAN_RxHandler handler)
{
    if ((bus >= 1u) && (bus <= 2u))
    {
        S_Rx_Handler[bus] = handler;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t d[8];
    const uint8_t bus = (hfdcan->Instance == FDCAN2) ? 2u : 1u;
   

    /* 循环读空：一次中断把所有待处理帧都消费掉 */
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, d) == HAL_OK)
    {
        /* 长度检查放在 bsp：本工程所有合法报文都是 8 字节，
         * 短帧只可能来自异常节点，直接丢，避免上层拿“残留的旧字节”当有效数据。
         * 这样上层解析函数就不需要再传 len 参数。 */
        if (rx.DataLength < FDCAN_DLC_BYTES_8)
        {
            continue;
        }

        if (S_Rx_Handler[bus] != 0)
        {
            S_Rx_Handler[bus](bus, rx.Identifier, d);
        }
    }
}
