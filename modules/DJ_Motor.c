#include "DJ_Motor.h"
#include "main.h"
#include "bsp.can.h"
#include "fdcan.h"
/*
 *@brief :发送控制底盘电机的电流报文（id2，3留0）
 */
HAL_StatusTypeDef DJ3508_Send_I_Can2(int16_t ID1_I, int16_t ID2_I, int16_t ID3_I, int16_t ID4_I)
{
    const FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier = 0x200 ,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = DISABLE,
        .BitRateSwitch = DISABLE,
        .FDFormat = DISABLE,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0
    };
      
    int16_t I_Raw[4] = {ID1_I, ID2_I, ID3_I, ID4_I};
    
      for(uint8_t i = 0; i < 4; i++) 
    {
        if(I_Raw[i] > 16384)
        { 
            I_Raw[i] = 16384;
        } 
        else if (I_Raw[i] < -16384)
        {
            I_Raw[i] = -16384;
        }
    }

     uint8_t  TxData[8] = {0};
     
     for(uint8_t i = 0; i < 4; i++)
    {
       TxData[2 * i] = (uint8_t)((I_Raw[i] >> 8)& 0xFF);
       TxData[2 * i + 1] =(uint8_t)((I_Raw[i])& 0xFF); 
    }
        return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData );
}

/* 
 * @brief: 发送6020控制yaw报文
 */
HAL_StatusTypeDef DJ6020_Send_V_Can2(int16_t ID1_V, int16_t ID2_V)
{
    const FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier = 0x1FF ,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = DISABLE,
        .BitRateSwitch = DISABLE,
        .FDFormat = DISABLE,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0
    };
   
     uint8_t  TxData[8] = {0};
     
     if(ID1_V > 25000) 
    {
        ID1_V = 25000;
    }
     if(ID1_V < -25000)
    {
        ID1_V = -25000;
    }   
   
     if(ID2_V > 25000)
    {
        ID2_V = 25000;
    }
     if(ID2_V < -25000)
    {
        ID2_V = -25000;
    }

    TxData[0] = (uint8_t)((ID1_V >> 8 ) & 0xFF);
    TxData[1] = (uint8_t)(ID1_V & 0xFF);
    TxData[2] = (uint8_t)((ID2_V >> 8 ) & 0xFF);
    TxData[3] = (uint8_t)(ID2_V & 0xFF);

        return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData );
}
/*
 *@ brief: 控制两个摩擦轮转动
 */
HAL_StatusTypeDef DJ3508_Send_Can1(int16_t ID2_I, int16_t ID3_I)
{
    const FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier = 0x200 ,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = DISABLE,
        .BitRateSwitch = DISABLE,
        .FDFormat = DISABLE,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0
    };

     uint8_t  TxData[8] = {0};

     int16_t I_Raw[2] = {ID2_I, ID3_I};
    
      for(int8_t i = 0; i < 2; i++)
       {
          if(I_Raw[i] > 16384)
         {
            I_Raw[i] = 16384;
         }
          if(I_Raw[i] < -16384)
         {
            I_Raw[i] = -16384;
         }
       }

            TxData[2] = (uint8_t)((I_Raw[0] >> 8)& 0xFF);
            TxData[3] = (uint8_t)((I_Raw[0])& 0xFF);
            TxData[4] = (uint8_t)((I_Raw[1] >> 8)& 0xFF);
            TxData[5] = (uint8_t)((I_Raw[1])& 0xFF);

        return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData );
}

/* ---------------- CAN1：pitch（6020 ID2）---------------- */
HAL_StatusTypeDef DJ6020_Send_Can1(int16_t ID2_V)
{
    const FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier = 0x1FF ,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = DISABLE,
        .BitRateSwitch = DISABLE,
        .FDFormat = DISABLE,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0
    };

     uint8_t  TxData[8] = {0};

     if(ID2_V > 25000)
    {
        ID2_V = 25000;
    }
     if(ID2_V < -25000)
    {
        ID2_V = -25000;
    }

    TxData[2] = (uint8_t)((ID2_V >> 8 ) & 0xFF);
    TxData[3] = (uint8_t)(ID2_V & 0xFF);

        return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData );
}

volatile GM6020_State_t gim6020 = {0};
volatile GM6020_State_t gim_pitch = {0};
volatile GM6020_State_t dj_motor[4] = {0};   /* CAN1：摩擦轮 ID2/ID3 */
volatile GM6020_State_t dj_wheel[4] = {0};   /* CAN2：底盘四轮 ID1~ID4 */
/* ---- 诊断计数器（只读观察，不参与控制）---- */
volatile uint32_t can_rx_total   = 0;   /* 收到的总帧数 */
volatile uint32_t can_rx_205     = 0;   /* 其中 0x205 的帧数 */
volatile uint32_t can_rx_last_id = 0;   /* 最近一帧的 CAN ID */
volatile int16_t  can_rx_last_d0 = 0;   /* 最近一帧的 data[0] */


/* 处理6020反馈报文，解析电机状态*/

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t d[8];

    /* 循环读空：一次中断把所有待处理帧都消费掉 */
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, d) == HAL_OK)
    {
        can_rx_total++;
        can_rx_last_id = rx.Identifier;
        can_rx_last_d0 = d[0];

        if (rx.Identifier >= 0x201 && rx.Identifier <= 0x204)      /* 3508 ID1~ID4 */
        {
            uint8_t i = (uint8_t)(rx.Identifier - 0x201);

            /* ★ 两条总线上的 3508 反馈 ID 会撞车：CAN2 是底盘四轮 ID1~ID4，
             *   CAN1 是摩擦轮 ID2/ID3。必须按总线分流，否则两边互相覆盖，
             *   表现为“温度/转速乱跳”“摩擦轮温度误触发保护”。 */
            volatile GM6020_State_t *st = (hfdcan->Instance == FDCAN2) ? &dj_wheel[i]
                                                                       : &dj_motor[i];
            st->angle     = (uint16_t)(((uint16_t)d[0] << 8) | d[1]);
            st->rpm       = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
            st->current   = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
            st->temp      = d[6];
            st->last_tick = HAL_GetTick();
        }
        else if (rx.Identifier == 0x205)                 /* GM6020 ID1 = yaw */
        {
            can_rx_205++;
            gim6020.angle     = (uint16_t)(((uint16_t)d[0] << 8) | d[1]);
            gim6020.rpm       = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
            gim6020.current   = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
            gim6020.temp      = d[6];
            gim6020.last_tick = HAL_GetTick();
        }
        else if (rx.Identifier == 0x206)                 /* GM6020 ID2 = pitch */
        {
            gim_pitch.angle     = (uint16_t)(((uint16_t)d[0] << 8) | d[1]);
            gim_pitch.rpm       = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
            gim_pitch.current   = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
            gim_pitch.temp      = d[6];
            gim_pitch.last_tick = HAL_GetTick();
        }
    }
}


































