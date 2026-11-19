#include "DJ_Motor.h"
#include "main.h"
#include "bsp_can.h"
#include "fdcan.h"
/*
 *@brief :发送控制底盘电机的电流报文（）
 */
HAL_StatusTypeDef DJ3508_Send_I_Can2(int16_t ID1_I, int16_t ID2_I, int16_t ID3_I, int16_t ID4_I)
{
      
    int16_t I_Raw[4] = {ID1_I, ID2_I, ID3_I, ID4_I};
    /*限流处理*/
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
        return BSP_CAN_Send(2u, 0x200u, TxData, 8u);
}

/* 
 * @brief: 发送6020控制yaw报文
 */
HAL_StatusTypeDef DJ6020_Send_V_Can2(int16_t ID1_V, int16_t ID2_V)
{
   
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

        return BSP_CAN_Send(2u, 0x1FFu, TxData, 8u);
}
/*
 *@ brief: 控制两个摩擦轮转动
 */
HAL_StatusTypeDef DJ3508_Send_Can1(int16_t ID2_I, int16_t ID3_I)
{

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

        return BSP_CAN_Send(1u, 0x200u, TxData, 8u);
}

/* ---------------- CAN1：pitch（6020 ID2）---------------- */
HAL_StatusTypeDef DJ6020_Send_Can1(int16_t ID2_V)
{

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

        return BSP_CAN_Send(1u, 0x1FFu, TxData, 8u);
}

volatile GM6020_State_t gim6020 = {0};
volatile GM6020_State_t gim_pitch = {0};
volatile GM6020_State_t dj_motor[4] = {0};   /* CAN1：摩擦轮 ID2/ID3 */
volatile GM6020_State_t dj_wheel[4] = {0};   /* CAN2：底盘四轮 ID1~ID4 */
/* ---- 诊断计数器（只读观察，不参与控制）（调试排错时使用）---- */
volatile uint32_t can_rx_total   = 0;   /* 收到的总帧数 */
volatile uint32_t can_rx_205     = 0;   /* 其中 0x205 的帧数 */
volatile uint32_t can_rx_last_id = 0;   /* 最近一帧的 CAN ID */
volatile int16_t  can_rx_last_d0 = 0;   /* 最近一帧的 data[0] */
volatile uint32_t can_rx_205_bad    = 0;  /* |rpm|>600 被判“非 6020 帧”丢弃的 0x205 帧数 */
volatile int16_t  can_rx_205_bad_rpm = 0;  /* 最近一帧被丢弃时的 rpm 原始值 */
volatile uint32_t can_rx_id_cnt[8]  = {0}; /* 按 ID 分别的收帧计数：[0..4]=0x201..0x205 */
volatile uint32_t can_rx2_total    = 0;   /* 仅 CAN2 收到的帧数（CAN2 活性看门狗用，CAN1 不计） */


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
        if (hfdcan->Instance == FDCAN2)
        {
            can_rx2_total++;                       /* CAN2 专属活性计数（看门狗用） */
        }
        if ((rx.Identifier >= 0x201u) && (rx.Identifier <= 0x208u))
        {
            can_rx_id_cnt[rx.Identifier - 0x201u]++;   /* 按 ID 计数：0x201..0x208 → [0..7] */
        }

        if (rx.Identifier >= 0x201 && rx.Identifier <= 0x204)      /* 3508 ID1~ID4 */
        {
            uint8_t i = (uint8_t)(rx.Identifier - 0x201);

            /* ★ 两条总线上的 3508 反馈 ID 会撞车：CAN2 是底盘四轮 ID1~ID4，
             *   CAN1 是摩擦轮 ID2/ID3。必须按总线分流，否则两边互相覆盖，
             *   表现为“温度/转速乱跳”“摩擦轮温度误触发保护”。 */
             /*6020反馈帧结构体适用于3508*/
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
            int16_t rpm205 = (int16_t)(((uint16_t)d[2] << 8) | d[3]);

            can_rx_205++;
            /* ★ 合理性过滤：6020 物理极限 ~320rpm（空载），|rpm|>600 的 0x205
             *   不可能是 6020 发的。CAN 帧自带 CRC15，总线误码的帧根本进不了
             *   RX FIFO，所以这种帧只可能来自别的节点——典型是某个 C620 电调
             *   拨码拨到 ID5（其反馈帧也是 0x205，rpm 可到 ±8000，且它会把
             *   0x1FF 字节0~1 当电流指令收，正好是我们发 yaw 电压的槽位）。
             *   这种帧一个都不能进 gim6020：进去一帧，yaw 的超速兜底就把
             *   电压掐成 0（表现为“枪口无阻力、yaw 遥控不动”）。
             *   整帧丢弃（连 last_tick 都不更新），让 fresh 超时兜底按
             *   “真掉线”处理，同时计数留证。 */
            if ((rpm205 > 600) || (rpm205 < -600))
            {
                can_rx_205_bad++;
                can_rx_205_bad_rpm = rpm205;
            }
            else
            {
                gim6020.angle     = (uint16_t)(((uint16_t)d[0] << 8) | d[1]);
                gim6020.rpm       = rpm205;
                gim6020.current   = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
                gim6020.temp      = d[6];
                gim6020.last_tick = HAL_GetTick();
            }
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


































