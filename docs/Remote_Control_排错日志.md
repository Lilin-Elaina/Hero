# Remote_Control 固件排错日志

> 覆盖：2026-10-17 ~ 2026-11-18　｜　依据：本工程 4 个会话存档（1200+ 条用户消息）+ `git log`（13 个提交，最早 10-28）　｜　整理：2026-11-18

## 阅读约定

| 字段 | 含义 |
|---|---|
| **现象** | 当时看到的可见行为（尽量保留原话） |
| **根因** | 代码/硬件层面的真实原因，带 `文件:行` |
| **处理** | 实际改了什么，贴关键代码 |
| **结果** | 解决 / 部分解决 / 遗留 |

行号是**当时**的行号，重构后会漂移，按函数名搜更可靠。

## 0. 基线

| 项 | 内容 |
|---|---|
| 主控 | STM32H723VGT6，达妙 DM-MC-Board02 |
| 工程 | HAL + Makefile + CubeMX；`make -j8`；`D:\STM32CUBE\STM32CubeCLT_1.22.0\GNU-tools-for-STM32\bin` |
| 底盘 | 4× M3508 + C620，麦轮；CAN2 上 0x201~0x204 |
| 云台 | GM6020 直驱；yaw = CAN2 0x205，pitch = CAN1 0x206 |
| 摩擦轮 | CAN1 0x200 的 byte2~5；供弹 3508 = 0x207 |
| 遥控 | DR16 + DBUS（UART5 + DMA 空闲中断），18 字节 / 14 ms |
| IMU | 板载 BMI088，SPI2（SCK=PB13 / MISO=PC2_C / MOSI=PC1 / CS=PC0 acc、PC3 gyro）；**胶带竖直绑在枪架上，随 yaw/pitch 一起动** |
| 关键文件 | `remote/rc_dbus.c`（控制）、`remote/rc_dbus.h`（参数）、`modules/mecanum.c`（混控）、`modules/ins.c`（IMU）、`modules/DJ_Motor.c`（CAN 收发）、`bsp/bsp.can.c` |
| 主循环 | `Core/Src/main.c` 的 `while(1)` + `HAL_GetTick()` 做 1 ms 分支：`INS_Task(); RC_Control_Task();` |

---

## 1. 时间线总览

| 日期 | 事件 | 一句话结论 |
|---|---|---|
| 10-17 | 烧录 `No rule to make target` | CubeMX 模板没有 flash 目标 |
| 10-17 | `DJ_Motor.h: No such file` | 文件名拼错 + 没进 `C_SOURCES` |
| 10-17 | DT7 推杆无反应 / 上电 HardFault / CAN 锁死 | DMA 落 DTCM + ORE 无恢复 + 无 bus-off 恢复 |
| 10-19 | 通道与方向反复错 | 实测接收机与文档相反；符号叠了多层 |
| 10-19 | `GM_SAFE_NO_OUTPUT 1` | 调试开关忘了关，输出恒 0 |
| 10-19 | 6020 一帧反馈都没有 | CAN 支路插错口（不是软件） |
| 10-19 | 6020 只单向转 | 限位没包住上电角 |
| 10-20 | `undefined reference` | 改签名后 .h/.c/调用点不同步 |
| 10-20 | 只有左拨杆在中位才能动 | `sw_swap` 与 `armed` 判据打架 |
| 10-20 | 自审 6 个隐患 | RX FIFO 溢出、假安全开关、占位限位 |
| 10-20 | 新接 3 个设备全离线 | 分线板把一条 CAN 断成两段 |
| 10-21 | 摩擦轮反向、yaw 只有 200 | 符号未定；`volt_prev` 丢了 |
| 10-21 | pitch 负重下垂 | 纯开环，无重力补偿 |
| 10-21 | 供弹 ID7 与 0x1FF 抢帧 | 同一帧 ID 两处各发各的 |
| 10-22 | 46 行限幅 | `int16_t` 形参在钳位前已截断 |
| 10-22 | `TxData[5]` 写错 | ID3 低字节填成高字节 |
| 10-22 ~ 23 | 遥控失灵 | `sw_swap` 硬编码反了 |
| 10-23 | 两轮改四轮 | 两条总线 ID 撞车 |
| 10-24 | “18 字节没有拨轮” | 帧里确实没有 |
| 10-25 | 右摇杆同时控云台和底盘 | 通道共用 `ch[0]` + 左侧两轮反接 |
| 10-30 | J-Link 连错 / `No rule` | 脚本被 Makefile 重生成 |
| 10-30 ~ 31 | pitch 双环首版缺陷链 | 错宏、多 `}`、超时块、`| 1u` |
| 10-31 | 嗡嗡嗡 / 软墙卡 3700 / 4100 停顿 | 硬开关极限环、静差、粘滑 |
| 11-01 | Git 误点 Revert | `git revert` 反做恢复 |
| 11-02 | 碰一下电机才回中 | 越界闸门 1 ms 清状态 |
| 11-03 | 四轮速度 PI | 转子 rpm 与车轮差 19.2 倍 |
| 11-03 ~ 04 | 撞墙 + 同一角度卡顿 | 命令 4700 但机械止点 4477 |
| 11-04 | 周期性卡顿 | 抗饱和双向冻结 |
| 11-05 | 超调 + 回调慢 | 缺加速度前馈，积分攒太满 |
| 11-06 | 卡 4400/4480、跳 4200、yaw 疯转 | 静差、越界闸门、`RC_CH_GIMBAL 0` |
| 11-06 | 重力前馈、积分分离、stage 卡 0 | 静止力矩必须常驻 |
| 11-06 | pitch 波形（本会话起点） | 位置环带宽 |
| 11-06 | 回中时部分轮反向转 0.5 s | 速度环积分饱和 |
| 11-07 | 参考速度周期性归零、积分死锁 | DBUS 丢帧 fail-safe；堵转攒积分 |
| 11-11 | 重新规划 | 转做跟随 + 小陀螺 |
| 11-12 | IMU/SPI + CubeMX 事故 | 生成代码删掉 UART5 |
| 11-12 | 遥控只能控 pitch | CAN2 ACK 错误 |
| 11-13 | 跟随 + 小陀螺一天做完 | Mahony、世界系、符号接反 |
| 11-14 ~ 16 | FreeRTOS 评估 | 只讨论，未落地 |
| 11-17 | 遥控失联 90% | CAN 焊点藕断丝连 |
| 11-18 | 跟随尾段 2 s、向左下角漂 | 两环互伺服；四轮出力不等 |

---

## 2. 详细条目

### 2026-10-17 14:16 烧录 `No rule to make target 'download_jlink'`

**根因**：CubeMX 生成的 Makefile 只有 `all`/`clean`，没有烧录目标；第二次是把目标命名成 `flash`，而用户敲的是教程里的 `download_jlink`。

**处理**
```make
# Makefile:207-241
flash download download_jlink flash-openocd: all
	# 运行时生成 build/Remote_Control.flash.jlink
	# connect → r → h → loadfile → r → g，-ExitOnError 1
.PHONY: flash download download_jlink flash-openocd
```
另建 `.vscode/tasks.json`。

**结果**：解决。实测 J-Link V9.70 / SWD 4000 kHz / STM32H723VG，131072 B @848 KB/s。附带：JLink.exe 只有 ConsoleWindowClass，本来没有 GUI，`-NoGui 1` 是误加。

---

### 2026-10-17 14:24 `fatal error: DJ_Motor.h: No such file or directory`（main.c:25）

**根因**：三层叠加——`-I` 只有 Core/Inc+HAL+CMSIS；磁盘文件名拼成 `DJ_Moter.*`、`mecanam.*`，与 `#include` 自相矛盾；4 个新源文件根本没进 `C_SOURCES`。

**处理**
- 改名 `modules/DJ_Motor.c/.h`、`modules/mecanum.c/.h`；`DJ_Motor.h` 补 `#include "main.h"`、修 include guard 重名。
- `bsp/bsp.can.c:7-17` 初始化器 `.FilterConfig.FilterType` 改平铺、`.IsCalibration`→`.IsCalibrationMsg`。
- `Makefile:70/140` 加 4 个源文件与 `-Ibsp -Imodules -Iremote`。

**结果**：解决。另发现 `-ffunction-sections`+`--gc-sections` 把无人调用的新代码全裁掉（nm 证实），当时 `rc_dbus.c` 只是 37 行空占位。

---

### 2026-10-17 20:35 DT7 推杆无反应：4 个互相掩盖的 bug

**现象**：DBUS 一帧收不到；偶尔上电即 HardFault（CFSR=0x400 IMPRECISERR、IPSR=0x45=UART5、PC 停在 DCIMVAC 后的 isb）；CAN 一旦 bus-off 永久锁死（CCCR.INIT=1、TEC=248、LEC=5）。

**根因**
1. `dbus_buf` 落在 DTCM(0x20000200)，H7 的 DMA 访问不到 DTCM，第一字节就 TEIF。
2. `RC_StartReceive` 丢弃返回值、没有 `HAL_UART_ErrorCallback`，一次 ORE 就永久静默。
3. D-Cache 没开却无条件 `SCB_InvalidateDCache_by_Addr()`。
4. 没有 bus-off 恢复。

**处理**
```c
/* STM32H723xG_flash.ld:201 */
.dma_buffer (NOLOAD) : { ... } >RAM_D1

/* remote/rc_dbus.c:42-44 —— 落到 0x24000000 */
__attribute__((section(".dma_buffer"), aligned(32))) uint8_t dbus_buf[...];

/* rc_dbus.c:49 清 ORE + 查返回值 + 重试；:350 错误回调里重新开接收 */
/* rc_dbus.c:330 先判断 D-Cache 是否真的开着 */
if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) { SCB_InvalidateDCache_by_Addr(...); }

/* bsp/bsp.can.c:29 新增 FDCAN2_Start()；:44 错误回调 Stop→重配→Start */
```

**结果**：解决。附带修正：3508/6020 反馈是大端 `(d[0]<<8)|d[1]`；`TXBTO` 是每缓冲一位、不是计数器（3 个缓冲最大 0x7），曾据此误判。

---

### 2026-10-19 12:00 摇杆通道与方向（复发 3 次）

**根因**：实测接收机与官方文档相反（`ch[2]` 才是前后）；叠加 `M1_SIGN/M4_SIGN` 被反复改，符号叠了多层；最后确认“车头装反了”。

**处理**
```c
/* remote/rc_dbus.h:68-69 */
#define RC_CH_FWD   RC_Data.ch[2]
#define RC_CH_TURN  RC_Data.ch[3]
/* remote/rc_dbus.c:259-260 —— 只在这里反一次 */
float fwd = online ? -RC_Norm(RC_CH_FWD) : 0.0f;
```

**结果**：解决。教训：符号只反一处，先定坐标系，架空测试才权威。

---

### 2026-10-19 21:20 6020 yaw 装好后“右拨杆不好使”

**根因**：AI 加的调试开关 `rc_dbus.h:88 #define GM_SAFE_NO_OUTPUT 1`，`rc_dbus.c:338-340` 里 `#if GM_SAFE_NO_OUTPUT volt = 0.0f;` 把输出强制为 0。

**处理**：置 0。保护链在 `rc_dbus.c:302-343`：反馈超时 100 ms → RPM 限 200 → 角度限位+软区 300 → 斜率 200 → 安全开关。

**结果**：恢复。教训：调试开关必须默认关闭并写明。

---

### 2026-10-19 22:40 6020 一帧反馈都没有（约 70 分钟）

**现象**：J-Link 读 TXBTO=0、PSR LEC=5(ACK error)，但 3508 正常。

**根因**：J-Link connect 会 halt CPU，寄存器读数不可信；真因是 6020 的 CAN 支路插错口 / `CAN_H-L` 接反，器件根本没上总线。

**处理**：`modules/DJ_Motor.c` 加诊断计数 `can_rx_total / can_rx_205 / can_rx_last_id`；实测 total 8465→9115（≈720 帧/s）而 `can_rx_205=0`，锁定“不在总线”。换口后通（97589 帧，angle 4671，27 ℃）。

**结果**：换口解决；随后接触不良导致 `last_tick` 冻结，重新插紧。教训：先看收帧计数，别信 halt 状态下的寄存器。

---

### 2026-10-19 23:51 6020 “转很慢、到角度就不动、只有右移才转”

**根因**：实测 `angle=4083`，而 `GM_ANGLE_MIN 5000`/`GM_ANGLE_MAX 7000`——当前角在限位外，越界逻辑把一侧电压清零；`GM6020_VOLT_MAX` 只有 8000。

**处理**：`rc_dbus.h` MIN 3060 / MAX 5100（±1024≈45°）、`GM6020_VOLT_MAX 15000`、`GM_RPM_LIMIT 350`、`GM_VOLT_SLEW 300`、`GM_SAFE_NO_OUTPUT 0`。

**结果**：解决。教训：限位必须包住上电角，否则单方向死锁。

---

### 2026-10-20 20:06 “文件全没了”——改名不同步（复发 3 次）

**现象**：`undefined reference to 'DJ6020_Send_I'`，用户以为工程被删（实际 28 个关键文件全在、0 字节文件 0 个）。

**根因**：`.c` 已改成 `DJ6020_Send_V(int16_t, int16_t)`，`DJ_Motor.h:8` 与 `rc_dbus.c:343` 仍是旧名旧参。

**处理**：头文件同步签名，调用点改 `DJ6020_Send_V((int16_t)volt, 0)`。

**结果**：解决；10-22 又因同类问题报 `conflicting types`。

---

### 2026-10-20 21:30 底盘只有左拨杆在中位才能动

**根因**：`RC_Decode` 里 `sw_swap` 判定写错、`s_left/s_right` 命名与实际相反，而 `rc_dbus.c:232` 用的是 `armed = (s_right == 3)`。

**处理**：`RC_Init()` 里置 `sw_swap = 1`（配 `RC_ENABLE_USE_LEFT`）。

**结果**：解决。

---

### 2026-10-20 22:03 自审出的 6 个隐患

| 隐患 | 处理 |
|---|---|
| RX 回调重复 `else if` 编译不过 | 合并分支 |
| `GM_SAFE_NO_OUTPUT` 只清 yaw 不清 pitch/摩擦轮（假安全开关） | 补齐 |
| `Core/Src/fdcan.c:57` `RxFifo0ElmtsNbr = 3`，对 6 节点×1 kHz=6000 帧/s 必溢出 | 改 16，回调 `while (HAL_FDCAN_GetRxMessage(...) == HAL_OK)` 排空 |
| pitch 占位限位 3200/5200 会把上电在外的电机单方向锁死 | 先 0/8191 |
| `FRIC_TIMEOUT_MS` 因 `last_tick=0` 一上电就锁死 | 放大到 5000 |

**结果**：全部修完；前 3 项正是后来“设备全离线 / 摩擦轮不动”的直接原因。

---

### 2026-10-20 23:43 分线板把一条 CAN 拆成互斥两段 → 改双总线

**现象**：新接 3 个设备全不在线，总线上只有 0x201/0x204/0x205，缺 0x202/0x203/0x206；`dj_motor[1..2]`、`gim_pitch` 全 0、`last_tick=0`。

**根因**：一块分线板把一条总线断成两段，接 A 就断 B；保护逻辑因 `last_tick=0` 清零输出（保护本身正确）。

**处理**：CAN2(PB5/PB6) = 底盘 ID1/ID4 + yaw；CAN1 = 摩擦轮 ID2/ID3 + pitch。三个坑：
- `fdcan.c:66` `TxFifoQueueElmtsNbr` CubeMX 生成 0 → CAN1 一帧都发不出，手改 3。
- `fdcan.c:55` `MessageRAMOffset=0x400`（10 KB 消息 RAM 共享）。
- PA11/PA12 在板上是 USB 引脚、没有收发器（TEC=48、LEC=5），换 PD0/PD1 才通。

**结果**：双总线通。另：CubeMX 重新生成会清空 `main.c` 里的 `FDCAN*_Start()/RC_Init()`，必须写进 USER CODE 段。

---

### 2026-10-21 18:49 摩擦轮 / pitch 上双总线 + 主动刹车

**根因**：内外轮符号未定；松杆只是把目标降 0，靠自然阻尼减速超慢；`volt_prev` 更新被删导致 yaw 电压只有 200。

**处理**
```c
DJ3508_Send_C1(ID2, ID3);   /* DJ_Motor.c:116 */
DJ6020_Send_C1(ID2_V);      /* DJ_Motor.c:162 */
/* rc_dbus.h: FRIC_L_SIGN -1（内→外）/ FRIC_R_SIGN +1 / FRIC_RAMP_A_PER_MS 0.02
             FRIC_TEMP_LIMIT 60 / FRIC_BRAKE_A 0.8 / FRIC_BRAKE_RPM 300 / FRIC_USE_CAN1 1 */
volt_prev = volt;           /* 补回 */
```

**结果**：解决。

---

### 2026-10-21 18:57 pitch 负重下垂 → 位置闭环 + 重力前馈

**根因**：纯电压开环，没有重力补偿。

**处理**：`GM_PITCH_LEVEL 4096`、`GM_PITCH_RANGE 1024`、`KP 25 / KI 0.03 / KD 0`、`I_LIMIT 8000`、`GRAV_K 1500`、`GM_COUNTS_PER_DEG = 8192/360 ≈ 22.76`；误差按 ±4096 处理 0/8191 回绕。

**结果**：能用；`LEVEL` 至今未标定（遗留）。

---

### 2026-10-21 19:30 供弹 3508(ID7) 与 0x1FF 抢帧

**根因**：ID7 走 `0x1FF` slot2(byte4-5)，而 6020 电压帧也用 `0x1FF`，两处各发各的会互相覆盖。

**处理**：`FEED_PERIOD_MS 10000 / FEED_ANGLE 1024（8 档 45°）/ FEED_I_A 2.0 / FEED_RPM_MAX 300 / FEED_TIMEOUT_MS 3000`；把 0x1FF 合并成一帧统一发（反馈 0x207）。

**结果**：通；`FEED_ANGLE` 仍需实测。

---

### 2026-10-22 19:22 46 行限幅 → `clamp_i16`

**根因**：4 个通道各 8 个 `if` 重复；且 `int16_t` 形参在钳位前就已经截断（40000→-25536），钳位救不回来。

**处理**
```c
/* modules/DJ_Motor.c —— 用函数不用宏：宏会二次自增、无类型检查 */
static inline int16_t clamp_i16(int32_t v, int32_t lo, int32_t hi)
```

**结果**：46 行→6 行，零警告；建议形参改 `int32_t`。

---

### 2026-10-22 21:53 编译冲突 + `TxData[5]` 真 bug

**现象**：`conflicting types for 'DJ6020_Send_V_Can2'`、`implicit declaration of DJ3508_Send_I / DJ6020_Send_V / *_C1`。

**根因**：又是 `.h`(两参)/`.c`(一参)/调用点三处不同步；另 `DJ_Motor.c:115` `TxData[5] = (I_Raw[1] >> 8)` 把 ID3 的低字节填成了高字节，摩擦轮右轮电流值错。

**处理**
```c
DJ_Motor.c:49  补 ID2_V + ±25000 钳位填 TxData[2..3]
DJ_Motor.c:115 改 (I_Raw[1] & 0xFF)
rc_dbus.c:444-450 全换 *_Can2 / *_Can1
```

**结果**：`-Wall` 零警告零错误，elf 生成。提醒：CAN2 的 0x1FF slot1 每 1 ms 发 0，将来挂 C620 ID5-8 会撞车。

---

### 2026-10-22 ~ 10-23 遥控失灵：两根拨杆认错

**现象**：“现在检查下代码，为什么遥控器失灵了”“重启几次表现不一样”；两轮电流恒 0，摩擦轮却能开。

**根因**
- `remote/rc_dbus.c:60` 硬编码 `RC_Data.sw_swap = 1u;`，把物理左拨杆当右。
- `remote/rc_dbus.c:233` `uint8_t armed = (RC_Data.s_right == RC_ENABLE_SW_POS);` 恒 0 → `enabled=0` → `modules/mecanum.c:72-78` 清 vx/vy/wz。
- `RC_SwitchAutoName()`（`remote/rc_dbus.c:85-140`，写入点 `:137`）在上电 3 s 内又把“唯一离中位 1 s 的拨杆”判为物理左并锁定。

**处理**
```c
RC_Data.sw_swap = 0u;
RC_Data.sw_auto_done = 1u;   /* 锁死，别再自动命名 */
```

**结果**：解决。

---

### 2026-10-23 两轮差速 → 四轮麦轮：两条总线 ID 撞车

**根因**：CAN1 与 CAN2 上的 3508 反馈同为 0x201~0x204，原解析只按 ID 写 `dj_motor[]`，互相覆盖；差分逆解也不适用麦轮。

**处理**
```c
extern volatile GM6020_State_t dj_wheel[4];   /* DJ_Motor.h：按总线分流 */
DJ3508_Send_I_Can2(raw[0], raw[1], raw[2], raw[3]);   /* 底盘全走 CAN2 */
if (rx.Identifier >= 0x201 && rx.Identifier <= 0x204) { /* CAN2 → dj_wheel */ }
```

**结果**：解决；失联时四轮硬置零。

---

### 2026-10-24 “18 字节里没有拨轮”

**根因**：手册 bit128~143 是**保留字段**，`RC_Ctl_t` 里没有 wheel；按结构算 20 字节 ≠ `RC_FRAME_LENGTH 18`。“byte16~17 是拨轮”来自开发板例程。DBUS 每 14 ms 发 18 字节后静默，`HAL_UARTEx_ReceiveToIdle_DMA(..., sizeof(dbus_buf))` 的 `sizeof=36` 只是防溢出上限，回调里的 `Size` 是累计字节数。

**处理**
```c
RC_Decode(&dbus_buf[Size - DBUS_FRAME_LEN]);   /* 取尾 */
```
**遗留**：`remote/rc_dbus.c:484` 的 `Size >= DBUS_FRAME_LEN` 应改 `==`，否则收 19 字节会解出错位帧。

---

### 2026-10-25 右摇杆一推，云台和底盘同时动

**根因**：`RC_CH_GIMBAL` 与 `RC_CH_STRAFE` 共用 `RC_Data.ch[0]`；左侧两轮物理反转，叠加后横移列变成四轮同号。

**处理**：`modules/mecanum.c:22-25` 横移列改 `(-,-,+,+)`；`remote/rc_dbus.h:58` `#define RC_CH_GIMBAL RC_Data.wheel`。

**结果**：解决。整体改 `M1/M4_SIGN=-1` 的方案因“摇杆跟着坏”被撤回。

---

### 2026-10-30 J-Link 连错目标 / `No rule to make target 'flash'`

**根因**：`build/Remote_Control.flash.jlink` 每次被 `Makefile:232-241` 重新 echo 生成，手改无效；CubeMX 模板没有 `flash` 目标，且 Makefile 每次生成都被覆盖。

**处理**
```make
JLINK_SN ?=
JLINK_FLAGS += -SelectEmuBySN $(JLINK_SN) -ExitOnError 1
jlink-list: ...
```
```bat
setx MAKEFILES "D:\STM32CUBE\flash.mk"
```
（`setx` 写 HKCU 被拒，需用户自己执行。）

**结果**：解决。

---

### 2026-10-30 ~ 10-31 pitch 双环首版缺陷链

**现象**：不抗重力、不回中，编译报 `remote/rc_dbus.c:352:1: error: expected identifier or '(' before '}' token`。

**根因（一条链）**
1. 宏名混用：`GM6020_VOLT_MAX` 应为 `GM_PITCH_VOLT_MAX`（`:309/317`）。
2. `:339` 已闭合 `RC_Pitch_Control()`，`:352` 多一个 `}`。
3. 双环被包进 `if (s_stage == 1)`；回中超时块（`:296-302`）却在外面。
4. `s_home_since = HAL_GetTick() | 1u;` → tick 100→101 相减下溢，立即超时。
5. 回中判据误用 `GM_PITCH_HOME_RATE`（500）当角度容差。

**处理**
```c
s_stage = (GM_PITCH_HOME_ENABLE && !s_homed) ? 1 : 2;
if (s_stage == 1) {
    if (s_home_since == 0u) s_home_since = HAL_GetTick();   /* 去掉 | 1u */
    ...
}   /* 超时块整体移进 stage1 */
/* 到位判定：pitch_abs(cur - home) <= 容差 */
```

**结果**：解决（`I_MAX 12000→15000`）。

---

### 2026-10-31 09:43 回中失败即锁死

**根因**：回中失败后 `s_ref` 停在半路，`s_homed` 又置 1，再也不回中。

**处理**
```c
s_ref = cur; s_i = 0; s_homed = 1; s_stage = 2; s_home_since = 0;   /* 就地按住 */
if (s_stage == 0 && pitch_abs(stick) > GM_PITCH_STICK_DEAD) { ... }  /* 接管只看摇杆 */
```

**结果**：解决。

---

### 2026-10-31 19:46 近下限位“嗡嗡嗡”

**现象**：4663~4669 抖动，毛刺 0.9 s、50~80 counts。

**根因**：限位处每 1 ms 硬开关 + `s_i = 0.0f` 形成极限环，电流被斩成 1 kHz 方波。

**处理**：判定不是调参问题，改软墙。**结果**：抖动解决。

---

### 2026-10-31 19:47 软墙后卡在 3700

**根因**：`s_ref` 钳在 3680，软墙持续衰减向下力，位置环无 I → 留 20 counts 静差（机械下止点 3674）。

**处理**：`remote/rc_dbus.c:351-355` 改四段衰减，`const float soft = 60.0f;`

**结果**：解决。

---

### 2026-10-31 19:57 摇杆不丝滑 / 4100 停顿

**根因**：位置 P 要攒 44 counts（1.9°）才有 11 rpm；DBUS 14 ms 阶梯；4100 是摆臂水平位，`τ = G·cosθ` 最大，脱静摩擦需 300~500 电压单位，而积分只有 0.65/ms。

**处理**
```c
s_stick_f += (stick - s_stick_f) * 0.08f;          /* :258 声明、:285-287 */
v_ff = s_stick_f * GM_YAW_STICK_RATE / 136.53f;
volt += v_ff * 94.0f;                               /* :359 之后 */
```

**结果**：改善。当时报 `s_stick_f undeclared`（`:343`）说明烧的是旧固件——与后来 `Copy-Item` 保 mtime 的坑同源。

---

### 2026-10-31 20:14 ~ 23:19 自转/横移通道再错一次

```c
/* remote/rc_dbus.h:55-56 */
#define RC_CH_TURN    RC_Data.wheel
#define RC_CH_STRAFE  RC_Data.ch[3]
```
复发“拨杆有时候左有时候右”→ 真因是 `:60` 写了 `sw_swap` 但没置 `sw_auto_done`，`RC_SwitchAutoName()` 上电 3 s 内又改回来。Init 里补 `sw_auto_done = 1u`（`:56-62`）后解决。

---

### 2026-11-01 11:56 宏映射混乱（运动学符号 vs 接线极性）

**根因**：`M1_SIGN` 混用运动学符号与接线极性，且乘在整轮总输出上（`remote/rc_dbus.c:469`）。

**处理**：拆两层——`modules/mecanum.c:22-25` 纯几何；`remote/rc_dbus.h:36-44` 纯硬件 `MOTOR_POL`。

**结果**：解决。

---

### 2026-11-01 16:59 Git 面板误点 Revert

**根因**：`4f1e6db Revert "注释更正与pitch pid初调"` 把 `416fda5` 反向；revert 不删原提交，reflog 仍在。

**处理**
```bash
git revert --no-edit 4f1e6db      # → 9dc9035 Reapply，52 files / 9973 insertions
arm-none-eabi-gcc -fsyntax-only ...   # EXIT=0，与 16:39 保存逐字节一致
```

**结果**：完全恢复，推送 `github.com/Lilin-Elaina/Hero`。

---

### 2026-11-02 22:20 上电要碰一下电机才回中

**根因**：`cur < GM_PITCH_ANGLE_MIN(3600) || cur > GM_PITCH_ANGLE_MAX(4700)` 每 1 ms 清 stage/积分并 `return 0`；断电下垂或首帧 `angle=0` 就永久卡死。

**处理**：放宽范围 + 丢首帧。**结果**：解决。

---

### 2026-11-03 20:00 四轮速度 PI

**根因**：`dj_wheel[i].rpm` 是**转子**转速，减速比 19.2:1，`RC_WHEEL_RPM_MAX 1000` 差一个数量级（0.5 m/s = 2412 rpm）；`Mecanum_Mix` 里 `float step` 被复用，任一轮过零则四轮全切快档。

**处理**
```c
#define RC_WHEEL_RPM_MAX   1000.0f
#define RC_WHEEL_KP        0.002f
#define RC_WHEEL_KI        0.002f
RC_Wheel_SpeedPI(i, ref, rpm, stop_cmd, limit_a);
raw[i] = (int16_t)(a / 20.0f * 16384.0f);
```
另：`RC_ABS_CURRENT_A = 8.0f`（额定 80%）、温度 60 ℃、堵转默认关；松杆清 `mec_prev`；删独立制动统一 `ref=0`；斜坡改向量插值。

**结果**：对照手册逐项核对不越界，可上板；三轮驱动会跑偏，仅调试。

---

### 2026-11-03 21:47 ~ 11-04 10:27 pitch 撞墙 + 同一角度卡顿

**根因**：上止点实测 4477，而命令 4700 → 190 counts ≈ 8.4° 一直顶墙、积分饱和；卡顿在 7 个周期里固定出现在同一角度（升 4.35k、降 4.05k，斜率减半）→ 与角度相关，手推顺滑已排除机械。

**处理**：`ANGLE_MIN 3590 / MAX 4430`；若属摩擦加 `GM_PITCH_FRIC_FF 800.0f`。

**结果**：限位待改；卡顿归控制侧，未闭环。

---

### 2026-11-04 02:35 pitch 周期性卡顿（抗饱和）

**根因**：`remote/rc_dbus.c:347-351` 抗饱和是**双向冻结**——饱和时连退饱和方向也冻住，`s_i` 停在 ±15000；换向要 `|err| > 100 rpm` 才跨零，前 100 ms 只爬到 ~37 rpm。

**处理**
```c
if ((volt > -GM_PITCH_VOLT_MAX && volt < GM_PITCH_VOLT_MAX) ||
    (volt >= GM_PITCH_VOLT_MAX && err < 0.0f) ||
    (volt <= -GM_PITCH_VOLT_MAX && err > 0.0f))
{
    s_i += GM_PITCH_SPD_KI * err * 0.001f;
}
```

**结果**：解决。

---

### 2026-11-05 12:36 超调且回调慢

**根因**：位置环纯 P，参考角以 1500 counts/s 匀速走 → 稳态滞后 `1500/(136.53×0.25) ≈ 44`；`KI 650` 每 ms 攒 28.6，0.5 s 撞 `I_MAX 15000`，泄到 1500 需 ≈415 ms。

**处理**
```c
const float v_ff = (s_ref - s_ref_prev) * 1000.0f / 136.53f;
rpm_d += v_ff;
/* I_MAX 15000→3000，SPD_KI 650→200 */
```

**结果**：改善；13:02 用户要求回退对比，13:31 恢复。

---

### 2026-11-05 13:48 回中不触发 / 只到 4500

**根因**：`KI 750→200` 后攒 1200 码需 1.6 s，加行程 1.58 s ≈ 3.2 s > `HOME_TIMEOUT 3000 ms`；`s_homed` 是 static，一置 1 永不复位（超时分支也置 1）→ 每次上电只回中一次。

**处理**：`KI 200→700`、`HOME_RATE 500→800`、`TIMEOUT 3000→5000`；掉使能分支补 `s_homed = 0;`。

**结果**：部分解决，仍卡 4400~4480。

---

### 2026-11-06 01:46 回中卡 4480（上次 4400）

**根因**：机械硬限位每次应该同数，两次不同＝放弃那一刻停在哪就保持在哪，`s_ref` 从未到 4100。

**处理**：断电手推 → 手动压杆 → Ozone watch `RC_Pitch_Control::s_stage` / `s_ref`（必须加载 `build/Remote_Control.elf`，静态局部变量带函数名前缀）。

**结果**：定位，未修完。

---

### 2026-11-06 02:06 回中后跳到 4200

**根因**：静差 `counts = s_i/(100×POS_KP) = 1500/16 ≈ 94`，是**静态平衡**，等 10 s 无效（4480 那次反推 `s_i ≈ 1230`）。

**处理**：`POS_KP 0.16→0.30`，或判定窗 60→120。**结果**：治标；根因是积分/前馈配比。

---

### 2026-11-06 02:51 一插遥控 yaw 疯转

**根因**：`remote/rc_dbus.h:72` 写成 `#define RC_CH_GIMBAL 0`。`RC_Norm` 输出 11 bit 值（364~1684，中位 1024），`0` 是**超量程最小值** → `-1024 + 死区` → `/660` → `clamp → -1.0`，volt 满 15000，经 `GM_VOLT_SLEW 200/拍` 75 ms 到顶。

**处理**：改回 `RC_Data.wheel`。**结果**：解决。教训：**0 不是零输入**。

---

### 2026-11-06 09:45 重力前馈

**根因**：静止时 `volt = s_i`，`I_MAX` 必须大于顶重力所需电压（≈3900）。

**处理**
```c
volt = pitch_clamp(GM_PITCH_SPD_KP * err + s_i + GM_PITCH_GRAV_FF, ...);
/* GM_PITCH_GRAV_FF 3900；I_MAX 15000→2000→10000 */
```
**结果**：解决（“积分上限提到 10000 就解决了卡 4400”）。

---

### 2026-11-06 10:50 超调复发

**根因**：追赶段积分一路累积。

**处理**：先试积分分离 `if (pitch_abs(err_pos) < GM_PITCH_I_ZONE)`，但位置误差门会让积分变成“路径记忆”、单向消差 → 改**速度门**：按实测 rpm 冻结/放开，撤掉 `err_pos` 与 `I_ZONE`。

**结果**：解决。教训：有 `GRAV_FF` 才能做积分分离；门判据必须用 rpm。

---

### 2026-11-06 11:56 stage 一直卡 0

**根因**：只有两处能把 stage 打回 0——`:268` 的 `!CALIBRATED || !online || !armed`，`:281` 的 `last_tick / temp>65℃ / |rpm|>GM_RPM_LIMIT / 角度越界`。`CALIBRATED=1`、`armed` 写死 1，只剩 online/temp/last_tick/越界。判据 `pitch_abs(cur-home) <= 60.0f` 常过不去（静差 46~560）→ 3 s 超时后就地放弃。

**处理**：`:332` 60→150；`ANGLE_MARGIN 80→40`（否则生效上界是 4670 而不是 4710）；底部 `lo = 3450+80 = 3530` 与实测止点 3746 不符，是 `s_i` 饱和到 −10000 的原因。

**结果**：待验证。

---

### 2026-11-06 11:16 pitch 波形（本会话起点）

**现象**：波形图（绿=参考、红=实测），“云台很抖”。

**根因**：位置环与速度环带宽不匹配，参考斜坡拐点处“先滞后、再追上”；平台段是直线，说明不是高频自激。

**处理**：调 `GM_PITCH_POS_KP`；后续给 D 项单独限幅 `GM_YAW_ANG_D_MAX 40`（`0.60×250 rpm = 150 rpm` 会占满 `RPM_MAX`，导致“调 D 没效果”）。

**结果**：pitch 基本稳定，问题转到 yaw。

---

### 2026-11-06 21:35 回中时部分轮反向转 0.5 s

**根因**：速度环积分饱和。`I_MAX 6 A`、`KP 0.002` → 靠 P 顶回 6 A 需要 3000 rpm 误差，做不到，积分只能慢慢泄。

**处理**：`I_MAX 6→1.5`，`|ref| < 50` 时清 `s_wheel_i`（实测曾冲到 −5.2689）。

**结果**：解决。

---

### 2026-11-07 参考速度周期性归零 / 积分死锁

**根因**：DBUS 丢帧 → `RC_Data.online` 掉 → fail-safe 清零（保护逻辑，正常）；13:07 手挡轮子时误差持续为负，积分攒到 `I_MAX`，松手后 P 已过零、积分还在往同方向推。

**处理**：`I_MAX` 下调 + `|ref|` 小时清积分；加 `RC_GapMaxMs` 镜像便于观察。

**结果**：解决。

---

### 2026-11-11 21:33 隔了四天，重新规划

**现象**：“已经过了四天了，我已经忘了之前讨论什么了”。

**处理**：给两条路线（IMU 世界系 vs 纯编码器），确定用 BMI088 做世界系，自转时平移用枪口系旋转矩阵。

**结果**：方向确定。

---

### 2026-11-12 IMU/SPI 上板 + CubeMX 事故

**现象**：22:30“遥控器只能遥控 pitch 了”；22:36“TOTAI 在涨但是 205 不涨”；22:40“推杆 ch3 变化，hardfault 非 0，ref 为 0”；22:45“四个轮怎么全不动”；22:49“有 514、515、518”。

**根因**
- PB13 从 `UART5_TX` 让给 `SPI2_SCK` 后，CubeMX 把异步 UART5 整块删掉（含 `dma.c`/`usart.c`/中断）→ DBUS 没了。
- `.ioc` 里 UART5 没补回，**再点 GENERATE 会再删一遍**。
- 514/515/518 是 CAN2 的 ACK 错误计数（错误被动），不是 Bus-Off。

**处理**：手工补回 UART5 初始化、DMA、`UART5_IRQHandler`；反馈分支按总线区分。

**结果**：解决；`.ioc` 仍是坏的（遗留）。

---

### 2026-11-13 跟随 + 小陀螺

**现象**：跟随时有夹角就一直自转；小陀螺时 yaw 完全跟着底盘走。

**根因与处理（按时间）**
1. **稳向用速度积分挪 `s_yaw_ref`** → 陀螺零偏越过死区就永久挪目标角 → “过一会儿自己转”。改为航向角位置环，再改纯速度保持。
2. **`HOLD_K=0` 时关节被位置环钉在车体上** → `gim6020.angle` 不变 → α 不变 → 跟随环永远追不到 0。开 `GM_YAW_HOLD_K=2.0` 后跟随恢复。
3. **串级带宽倒挂**：跟随外环 `KP 0.030` 时 6~8/s，快于 yaw 内环 4/s → 来回摆。降到 `0.012`，内环提到 10/s。
4. **小陀螺独立控制器**：`RC_Yaw_Spin_Control()` + Mahony（`modules/ins.c` 的 `ins_ahrs_step`，`MAHONY_KP 10 / KI 0.008 / HALFT 0.0005`），发布 `INS_WorldYawRate` / `INS_WorldYaw`（倾斜补偿）。
5. **符号接反导致疯转**：板子 x 轴朝下（`a_x≈-9.8`），`g·v = -g0`，再取负变成 `+g0 = -ω_yaw` → 正反馈。改 `INS_WORLD_YAW_SIGN (+1.0f)`，加 `SPIN_YAW_RATE_MAX 600` 与失控兜底。
6. **稳态被带偏**：`volt≈50·(w_cmd−ω)+I`，关节 200 °/s 需 ≈3133 电压单位，P 顶不住 → 加反电势前馈 `SPIN_YAW_KV 94`，`POS_KP 2.0 / KI 120 / I_MAX 4000`。

**结果**：跟随基本可用；小陀螺仍缓慢偏移（遗留）。

---

### 2026-11-14 ~ 11-16 FreeRTOS 评估

**处理**：纠正概念（它是调度器，不是控制环）；给两条路线（CubeMX 生成 / 手工挂 `Middlewares` + 改 Makefile + TIM6 时基）；分析参考工程 `RM_hero_ctype-241124`（5 个任务、`osDelay` 轮询、零 IPC、CubeMX 的 defaultTask 自杀）。

**结果**：只讨论，未落地。

---

### 2026-11-17 遥控失联 90% → CAN 焊点

**现象**：“遥控失联率 90%、所有电机很卡、yaw 不动、短时间后像游戏延迟一样执行”；“失联时手掰枪口无阻力”。

**根因**：CAN 焊点藕断丝连 + 胶带固定 → 信号断续 → `gim6020.last_tick` 不更新 → `yaw_kill` 把 yaw 输出清零（保护逻辑正确）。

**处理**：补焊 + 热缩管/扎带做应力释放 + 双绞；总线健康判据：`FDCAN2->ECR` 的 TEC/REC 应为 0、`PSR.BO` 不为 1、断电测 CANH-CANL ≈60 Ω。

**结果**：解决（用户自查确认）。

---

### 2026-11-18 跟随尾段 2 s / 顺时针大跟随时向左下角漂

**现象**：跟随收敛后还有约 2 s 小角度单向补追；顺时针大幅度跟随时整车向左下角运动。

**根因**
- 云台是关节位置保持（`s_yaw_ref` counts），世界稳向块 `GM_YAW_HOLD_K 2.0` 同时往 `s_yaw_ref` 里加修正 → 云台动 → α 变 → 底盘追 → 稳向再修正，**两环互相伺服**；且守世界使 `dα/dt = ω_c/(1+HOLD_K) = ω_c/3`，等效增益除 3、时间常数乘 3。
- 漂移：`MECANUM_DIR` 是标准 X 型，纯自转不触发单轮截断 → 不是混控算错；是四轮出力不等（转心偏移）或麦轮打滑/装反。

**处理**：`RC_FOLLOW_KP 0.012→0.020`；`RC_FOLLOW_MAX 0.80→0.70`。

**结果**：待复测。漂移给两个测试：架空纯自转看 `dj_wheel[0..3].rpm` 是否等幅；地面纯自转一圈看车心偏移。

---

## 3. 反复踩的坑

### 3.1 宏与通道
- `RC_CH_GIMBAL 0`：0 不是“零输入”，是 −1024 → 满输出。
- `sw_swap`、`RC_CH_FWD/TURN`、`MECANUM_DIR` 的符号列：只能实测，不能照抄文档。
- `RC_WHEEL_RPM_MAX` 把转子 rpm 当车轮 rpm（差 19.2 倍）。
- 运动学符号（`MECANUM_DIR`）与接线极性（`MOTOR_POL`）必须分两层。
- 同名帧 ID 两处各发各的（0x1FF 与供弹 ID7）。

### 3.2 static 状态
- `s_homed` 置 1 后永不复位 → 每次上电只回中一次。
- `sw_auto_done` 不置 → 上电 3 s 内自动命名又把配置改回去。
- `volt_prev` 丢了 → 斜率从 0 开始爬，输出只有 200。

### 3.3 限幅 / 抗饱和
- 双向冻结（饱和时连退饱和方向也冻）。
- 用位置误差当积分门（变成“路径记忆”）。
- D 项不限幅（`0.6×250 rpm` 占满输出）。
- 硬开关式限位 → 1 kHz 极限环。
- 限位没包住上电角 → 单方向死锁。

### 3.4 工具链 / 构建
- `Copy-Item` 保留 mtime → make 认为源比 `.o` 旧 → 跳过编译，烧的是旧固件（曾出现 `text 40068` 假回退）。
- `make clean` 删掉 `build/_tmp` → gcc `Error 3`；用 `.tmpbuild` 做 TMP。
- PowerShell 文本管道改源码（`Get-Content -Raw` / `-replace` / `Set-Content`）会把 UTF-8 中文写成 U+FFFD；**只能用 edit/write 工具**。
- CubeMX 生成会删掉被挪走引脚的外设（UART5），也会重写 Makefile 和 `main.c` 的 USER CODE 之外部分。

### 3.5 串级 / 带宽
- 内环比外环慢 → 振荡；跟随外环必须比 yaw 内环慢。
- 稳向与跟随同时作用于 α → 两个环互相伺服。

### 3.6 硬件层
- H7 的 DMA 访问不到 DTCM，缓冲区必须放 `RAM_D1`。
- 分线板会把一条 CAN 断成两段，接 A 断 B。
- 板上 PA11/PA12 是 USB，不是 CAN。
- 焊点藕断丝连 + 胶带固定 → 断续丢帧，表现为“PID 坏了”。

---

## 4. 未解决 / 待验证

| 项 | 状态 |
|---|---|
| pitch 上止点仍是占位值（实测 4477，命令 4700） | 遗留 |
| pitch `LEVEL` 未标定、`FEED_ANGLE` 未实测 | 遗留 |
| 11-04 “同一角度斜率减半”的卡顿 | 控制侧，未闭环 |
| 小陀螺长时间仍缓慢偏移 | 关节位置环版 `7a3a2b5` 未上车验证 |
| 顺时针大幅跟随时向左下角漂 | 待做架空/地面两个测试 |
| 跟随尾段 2 s 补追 | `KP 0.020` 待复测 |
| `Size >= DBUS_FRAME_LEN` 应为 `==` | 遗留 |
| 底盘 FF `0.0008 A/rpm` 未标定 | 遗留 |
| `.ioc` 里 UART5 未恢复 | 禁止点 GENERATE |
| 工作区 +219 行取证镜像未提交 | 待决定 |
