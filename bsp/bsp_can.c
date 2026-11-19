#include "main.h"
#include "bsp_can.h"
#include "fdcan.h"

/* CAN2 故障取证镜像（定义在 rc_dbus.c，这里只 +1，避免互相 include） */
extern volatile uint32_t RC_Can2BusOffCnt;
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
        RC_Can2BusOffCnt++;          /* ★计数留证：反复 Bus-Off = 物理层在出错，不是软件 */
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
 * 统一发送：把“帧头怎么填”这件事收在 bsp 层，modules 只管 id + 8 字节负载。
 * 帧头各字段与原来 modules/DJ_Motor.c 里手写的一致（标准帧/数据帧/8 字节/
 * 不切波特率/FD 关闭/无 TX 事件），所以字节与总线时序完全不变。
 * ------------------------------------------------------------------ */
HAL_StatusTypeDef BSP_CAN_Send(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t len)
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

    switch (len)
    {
        case 1u: hdr.DataLength = FDCAN_DLC_BYTES_1; break;
        case 2u: hdr.DataLength = FDCAN_DLC_BYTES_2; break;
        case 4u: hdr.DataLength = FDCAN_DLC_BYTES_4; break;
        case 5u: hdr.DataLength = FDCAN_DLC_BYTES_5; break;
        case 6u: hdr.DataLength = FDCAN_DLC_BYTES_6; break;
        case 7u: hdr.DataLength = FDCAN_DLC_BYTES_7; break;
        default: hdr.DataLength = FDCAN_DLC_BYTES_8; break;   /* 本工程一律 8 字节 */
    }

    return HAL_FDCAN_AddMessageToTxFifoQ(h, &hdr, (uint8_t *)data);
}

void BSP_CAN_Stop(uint8_t bus)
{
    HAL_FDCAN_Stop((bus == 1u) ? &hfdcan1 : &hfdcan2);
}

/* ------------------------------------------------------------------
 * 接收：唯一的 HAL 回调留在 bsp；取到帧后按总线交给注册者解析。
 * ------------------------------------------------------------------ */
volatile uint32_t can_rx_total    = 0;   /* 两路合计帧数 */
volatile uint32_t can_rx2_total   = 0;   /* 仅 CAN2 帧数（活性看门狗用） */
volatile uint32_t can_rx_last_id  = 0;   /* 最近一帧的 CAN ID */
volatile int16_t  can_rx_last_d0  = 0;   /* 最近一帧的 data[0] */
volatile uint32_t can_rx_id_cnt[8] = {0};/* 0x201..0x208 分别计数 */

static BSP_CAN_RxHandler s_rx_handler[3] = {0};   /* [1]=CAN1，[2]=CAN2 */

void BSP_CAN_RegisterRx(uint8_t bus, BSP_CAN_RxHandler handler)
{
    if ((bus >= 1u) && (bus <= 2u))
    {
        s_rx_handler[bus] = handler;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t d[8];
    const uint8_t bus = (hfdcan->Instance == FDCAN2) ? 2u : 1u;
    (void)RxFifo0ITs;

    /* 循环读空：一次中断把所有待处理帧都消费掉 */
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, d) == HAL_OK)
    {
        can_rx_total++;
        can_rx_last_id = rx.Identifier;
        can_rx_last_d0 = d[0];
        if (bus == 2u)
        {
            can_rx2_total++;                       /* CAN2 专属活性计数（看门狗用） */
        }
        if ((rx.Identifier >= 0x201u) && (rx.Identifier <= 0x208u))
        {
            can_rx_id_cnt[rx.Identifier - 0x201u]++;   /* 按 ID 计数 */
        }

        if (s_rx_handler[bus] != 0)
        {
            s_rx_handler[bus](bus, rx.Identifier, d, 8u);
        }
    }
}
