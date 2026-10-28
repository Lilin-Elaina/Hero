# Remote_Control 项目长期备忘

## 硬件(已手册核实, 勿改)
- 主控: 达妙 DM-MC-Board02, MCU=STM32H723VGT6, 晶振 HSE=24MHz。
- 电机: 2×RoboMaster M3508 + C620 电调, CAN 总线, C620 ID1(左轮)/ID4(右轮)。
- 遥控: DT7+DR16, DBUS(100000bps/8数据+偶校验/18字节/14ms)。达妙板 SBUS 口有硬件三极管反相→UART 不要开软件反相。
- CAN 波特率=1Mbps(PLL2Q=80MHz / Prescaler4 / (1+15+4)=1.0Mbps), 与 C620 要求一致, fdcan.c 时序正确。
- C620 协议: 0x200 帧控 ID1~4 电流(高字节在前, ±16384↔±20A)；反馈帧 ID=0x200+电调ID。

## 已知代码 bug(2026-10-17 审查, 待用户自己修)
1. `rc_dbus.c:292` 调 `DJI_Motor_SetCurrentA()` 但全工程未定义→链接报 undefined reference。`rc_dbus.c:23` `#include "dji_motor.h"` 与实际文件 `DJ_Motor.h` 名/大小写不符。
2. `main.c` 的 while(1) 是空的: 从不调用 `Filter_Init()`/`HAL_FDCAN_Start()`/`RC_Init()`/`RC_Control_Task()`→上电 CAN 不动、遥控不收、电机不转。
3. `remote/` 头文件名 `rc.dbus.h`(带点) 与 `rc_dbus.c` 里 `#include "rc_dbus.h"`(下划线) 不符→编译报找不到头文件。

## CAN 节点总表(2026-10-20 扩展后)
| 电机 | 驱动器拨码 ID | 控制帧槽位 | 反馈帧 |
|---|---|---|---|
| 底盘左 3508 | C620 ID1 | 0x200 byte0-1 | 0x201 |
| 摩擦轮左 3508 | C620 ID2 | 0x200 byte2-3 | 0x202 |
| 摩擦轮右 3508 | C620 ID3 | 0x200 byte4-5 | 0x203 |
| 底盘右 3508 | C620 ID4 | 0x200 byte6-7 | 0x204 |
| yaw 6020 | ID1 | 0x1FF byte0-1 | 0x205 |
| pitch 6020 | ID2 | 0x1FF byte2-3 | 0x206 |
- ★同帧合并原则: 同一标识符(0x200/0x1FF)每周期只能发一帧, 多函数分开发会让电调在 0/目标间抖振;
  必须 Send_All 合并。pitch 有机械限位(可用角度限位), yaw 发射机构连续旋转(不限位)。
- 摩擦轮两轮转向必须相反; DT7 纯遥控模式无键鼠输入(键鼠要 DR16 插 USB), 开关只能用拨杆:
  右拨杆上位=摩擦轮开。用户板子 s_right↔物理左拨杆(使能在用), s_left↔物理右拨杆。

## ★达妙 DM02 板 CAN 口引脚映射(手册P17板图实证, 2026-10-21)
- CAN1 座=PD0(RX)/PD1(TX), AF9; CAN2 座=PB5/PB6(底盘+yaw在用); CAN3 座=PD12/PD13=FDCAN3。
- ★FDCAN1 千万别用 PA11/PA12——板上这两个脚是 USB DM/DP! CubeMX 默认引脚是坑。
- 每个座有 CANx_120R 终端电阻跳线; CAN 收发器 5V 供电, 需 USB 或 24V 给板上电。
- H723 有 3 个 FDCAN 外设(FDCAN3_BASE=0x4000D400)。

## 调车注意(装配相关)
- `rc_dbus.h`: M1_SIGN/M4_SIGN 默认 +1, 转向反要改一侧为 -1。
- 使能开关默认=右拨杆中位(RC_SW_MID)才出力；`RC_ENABLE_SW_POS=0` 可跳过。
- `mecanum.c` 斜坡按 1kHz 设计, 需 1ms 周期调用 `RC_Control_Task`。

## 扩 4 轮前必查(2026-10-18 发现)
- `mecanum.c` 注释矩阵(:6-9) 与 4 轮 `MECANUM_DIR[]` 数组(:24-30) 符号有出入(V_LB spin、V_RB strafe/spin 对不上)。
- 当前 `MECANUM_WHEEL_NUM=2`, 4 轮分支被 #if 排除编译, 不影响运行。
- 扩 4 轮时必须按实物辊子朝向(X 型 / O 型)重新核对 `MECANUM_DIR[4]`, 别信注释矩阵。
- `MEC_ACCEL_NORM=1.5` / `MEC_STOP_NORM=4` / `MEC_SPIN_RATIO=0.7` / `MEC_DEADBAND_NORM=0.02` 是调车经验值(非手册规定), 可改。

## 6020 云台(2026-10-19, 已诊断)
- 3508 遥控已成功；6020 代码: `DJ6020_Send_I`(0x1FF)、回调解析 0x205→`gim6020`、CAN 接收中断链路全通。
- "右摇杆水平 6020 不转"的原因=`rc_dbus.h` `GM_SAFE_NO_OUTPUT=1`(安全开关, 强制 volt=0, 设计如此非 bug)。
- 正确启用顺序: 先实测 `gim6020.angle` 范围 → 填 `GM_ANGLE_MIN/MAX`(1200/7000 是占位值) → 再改 `GM_SAFE_NO_OUTPUT=0`。
- `GM_RPM_LIMIT=200` 偏低(6020 空载约 320rpm), 建议调 ~400。
- 6020 协议(0x1FF 控 ID1~4, 0x205 反馈, ±25000↔±25V, 角度 0~8191)已按官方手册核对全部正确。
- 发射机构是**连续旋转**重载关节(可跨 0/8191 过圈), 之前"限位 1000~2925/5000~7000"都不是真实机械行程;
  连续旋转不要加角度限位, 限位区间跨圈还需处理回绕。8V 电压对重载不够(电流 5570 才 8rpm), 建议 15000~18000。
- 用户澄清: 6020 转得慢是发射机构太重, 非链路问题; 方向/丝滑调 GM6020_SIGN、GM_VOLT_SLEW、GM_RPM_LIMIT(建议 400)。
