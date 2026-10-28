#include "main.h"
#include "bsp.can.h"
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
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) == 0 )
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
