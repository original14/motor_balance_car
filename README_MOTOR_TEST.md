# 天猛星 MSPM0G3507 双电机安全测试说明

## 1. 工程与工具链

- 工程：`motor`，CCS Theia MSPM0G3507 NoRTOS/TI Arm Clang 工程。
- 工程配置：MSPM0 SDK 2.11.0.07、SysConfig 1.26.2、TI Arm Clang 4.0.0 LTS。
- 调度：TIMG0 每 10 ms 进入控制 ISR。编码器取样、直接目标更新、PID 和保护都在该 ISR 中完成；主循环只解析命令并按 50 Hz 发送遥测。PID 直接使用每 10 ms 的编码器变化值，不计算 RPM，也不做速度滤波。UART 轮询发送可被控制 ISR 抢占，不会推迟控制周期。

## 2. 引脚与接线

| 功能 | MCU 引脚 | SysConfig 外设 |
|---|---:|---|
| TB6612 STBY | PA25 | GPIO 输出，初始低 |
| TB6612 AIN1 / AIN2 | PA8 / PA9 | GPIO 输出，初始低 |
| TB6612 PWMA（左轮） | PA26 | TIMG7_C0，20 kHz |
| TB6612 BIN1 / BIN2 | PA28 / PA31 | GPIO 输出，初始低 |
| TB6612 PWMB（右轮） | PA27 | TIMG7_C1，20 kHz |
| 左编码器 A / B | PB0 / PB1 | GPIOB 双边沿中断、内部上拉 |
| 右编码器 A / B | PA29 / PA30 | TIMG8 硬件 QEI（CCP0 / CCP1）、内部上拉 |
| 板载 USB 串口 TX / RX | PA10 / PA11 | UART0，115200 8N1，RX 中断 |
| 控制定时器 | 无外部引脚 | TIMG0，100 Hz |

左轮对应 TB6612 A 通道，右轮对应 B 通道。左编码器使用 PB0/PB1 双边沿 GPIO 中断做软件正交解码，右编码器继续使用 TIMG8 硬件 QEI。PA10/PA11 已连接板载 CH340，不能再接其他功能。PA26/PA27 的 TIMG7_C0/C1 复用已由 SDK 2.11 SysConfig 验证。MSPM0G3507 只有 TIMG8 支持硬件 QEI，PB2/PB3 不能复用为 TIMG8，因此右编码器必须从 PB2/PB3 改接到 PA29/PA30；PB2/PB3 在当前版本不再使用。

右编码器由 16 位硬件 QEI 连续累计，软件每 10 ms 用模 65536 差分并扩展为 64 位累计总数，因此计数器跨越 0/65535 时不会产生错误的大跳变，长时间高速运行也不会很快溢出。每次取样的绝对变化必须小于 32768，否则单次 16 位模差分无法判断真实方向。左编码器每个 A/B 边沿都进入 GPIO ISR；低速可用于对照测试，但高速时 CPU 中断负载大，可能因边沿过密而漏计。当前实测一圈约 60000 个 x4 计数只用于理解机械比例；PID 不使用 CPR，也不换算 RPM，而是直接把本周期变化值作为反馈。

TB6612 的逻辑电源 VCC 接 3.3 V，VM 接独立且满足电机规格的电机电源。开发板、TB6612、编码器和电机电源必须共地。严禁用开发板 3.3 V 给电机供电。建议 VM 使用限流电源，并在驱动板附近放置合适的储能/去耦电容。

编码器为 3.3 V 推挽输出时可直接接入。开漏输出建议外接 4.7 kΩ～10 kΩ 上拉至 3.3 V。5 V 编码器输出不能直接接 MSPM0，必须电平转换或合理分压；软件无法替代电平兼容检查。

## 3. 默认安全配置

`Config/motor_config.h` 当前设置：

```c
#define APP_RUN_MODE APP_MODE_PID_TEST
#define MOTOR_OUTPUT_MASTER_ENABLE 1
```

上电目标仍为 0，未收到非零命令时 STBY、两路 PWM 和四个方向脚均为低。需要只读编码器时，应先改为 `APP_MODE_ENCODER_TEST` 和总闸 0，再重新编译烧录。输出总闸为 0 时，驱动层的使能和 PWM 接口会强制紧急停止。任何 `STOP`、数值异常、编码器方向异常或无反馈故障都会清零目标与积分、PWM 归零、方向脚归零并拉低 STBY；`STOP` 会锁存，不能靠旧目标自动恢复。

四个独立方向宏：

- `LEFT_MOTOR_REVERSE`、`RIGHT_MOTOR_REVERSE`：只反转对应 TB6612 方向映射。
- `LEFT_ENCODER_REVERSE`、`RIGHT_ENCODER_REVERSE`：只反转对应编码器的累计总数和每 10 ms 变化值符号。

修改编译宏后必须重新编译并重新烧录才生效。

## 4. VOFA+ 与串口命令

VOFA+ 选择 FireWater 文本协议、115200、8N1。每行固定十六列、50 Hz、无日志和字段名。目标和 I 项输出保留两位小数，编码器变化值、实际 PWM 和累计总数输出整数，PID 增益保留四位小数：

1. CH0 左轮当前目标值（每 10 ms 期望编码器变化值，不是 RPM）
2. CH1 左轮参与 PID 计算的编码器变化值
3. CH2 左轮实际有符号 PWM 计数（不是百分比）
4. CH3 左编码器累计总数
5. CH4 左轮 Kp
6. CH5 左轮 Ki
7. CH6 左轮 Kd
8. CH7 左轮 I 项实际输出（`Ki × 累计误差积分`）
9. CH8 右轮当前目标值（每 10 ms 期望编码器变化值，不是 RPM）
10. CH9 右轮参与 PID 计算的编码器变化值
11. CH10 右轮实际有符号 PWM 计数（不是百分比）
12. CH11 右编码器累计总数
13. CH12 右轮 Kp
14. CH13 右轮 Ki
15. CH14 右轮 Kd
16. CH15 右轮 I 项实际输出（`Ki × 累计误差积分`）

命令支持 CR、LF、CRLF，不输出 ACK：

```text
LPID,<kp>,<ki>,<kd>
RPID,<kp>,<ki>,<kd>
LKP,<kp>
LKI,<ki>
LKD,<kd>
RKP,<kp>
RKI,<ki>
RKD,<kd>
LSPD,<target_delta>
RSPD,<target_delta>
STOP
```

`LKP/LKI/LKD` 和 `RKP/RKI/RKD` 只修改指定电机的一个增益并保留另外两个增益；运行中改参数不再清零累计误差积分和微分历史。`LSPD/RSPD` 命令名为兼容现有 VOFA 控件而保留，参数已经不是 RPM：发送 `RSPD,300` 会直接把 CH8 设为 `300.00`，表示期望右编码器每 10 ms 变化 300 个计数。目标必须是有限数且绝对值不超过 `MOTOR_TARGET_VALUE_LIMIT`（当前为 3000）；命令只有在 PID 模式、总闸开启且未 STOP 锁存时才会接受。

## 5. 三阶段测试

### 阶段一：手动测试编码器（必须首先执行）

执行该阶段前，先将 `APP_RUN_MODE` 设为 `APP_MODE_ENCODER_TEST`、将 `MOTOR_OUTPUT_MASTER_ENABLE` 设为 0；不要连接或不要开启 VM 电源。先手动向定义的正方向转左轮，确认 CH1 的每 10 ms 变化值为正且 CH3 累计增加；再转右轮，确认 CH9 为正且 CH11 累计增加。若符号错误，只修改对应的 `*_ENCODER_REVERSE`。输出轴恰好转一圈，可用 CH3/CH11 的累计变化量对比一圈计数；该数值不参与当前 PID 计算。

### 阶段二：判断电机方向

```c
#define APP_RUN_MODE APP_MODE_DIRECTION_TEST
#define MOTOR_OUTPUT_MASTER_ENABLE 1
```

轮子悬空并使用限流电源。上电等待 3 秒，两轮以 `+240` 的原始 PWM 计数运行 1 秒后永久停止。某轮方向错误时只改对应 `*_MOTOR_REVERSE`。不要同时反转电机与编码器宏来掩盖方向错误。最终应满足“正 PWM → 期望物理正方向 → 正 delta”。

### 阶段三：PID 调速

```c
#define APP_RUN_MODE APP_MODE_PID_TEST
#define MOTOR_OUTPUT_MASTER_ENABLE 1
```

默认目标仍为 0，不会自动启动。示例：

```text
RKP,0.5
RKI,0
RKD,0
RSPD,300
```

先保持 Ki=Kd=0 调 Kp；由于输入、反馈和输出单位均已改为原始计数，旧的 RPM/百分比参数不能直接照搬，需要重新整定。Kp 太小会跟不上，过大会振荡。Kp 合理后少量增加 Ki 消除稳态误差；Ki 太大会大幅振荡。通常先用 PI，只有 PI 合理后仍有快速超调才试很小的 Kd。随时发送 `STOP` 紧急停止。左轮现在使用 GPIO 中断编码器反馈，右轮使用硬件 QEI 反馈。

## 6. PID 与保护

PID 使用 `measurement = encoder_delta_10ms`、`error = target_delta_10ms - measurement`。累计误差按 `integralError += error × dt` 更新，I 项实际输出为 `integralOutput = Ki × integralError`，总输出为 `output = Kp × error + integralOutput + Kd × derivativeFiltered`。I 项输出按 PWM 输出范围限幅；总输出饱和且误差继续推向饱和方向时拒绝该周期积分（条件积分 anti-windup）。D 项使用测量微分 `-(measurement - previousMeasurement) / dt` 并一阶滤波，避免目标阶跃产生 derivative kick。算法中没有 RPM 换算。

PWM 定时器满量程是 1600 计数。当前 `LEFT_MAX_PWM_COMMAND` 和 `RIGHT_MAX_PWM_COMMAND` 均为 1280，所以 PID 的可用输出范围是 `-1280` 到 `+1280`；这是保留原先 80% 功率上限后换算得到的原始计数。CH2/CH10 显示经过限幅并四舍五入后实际写入 PWM 比较逻辑的有符号高电平计数，而不是 PID 限幅前的浮点结果，也不是百分比。若确实要开放硬件满量程，可在确认电机、电源和驱动安全后把两项上限改为 `1600.0f`。

PID 模式下分别监测左右轮：目标与 PWM 明显非零而编码器变化值连续反向，或目标与 PWM 非零却长期没有编码器变化，任一侧达到配置周期数即停止两轮并锁存故障。所有阈值均使用目标计数、每 10 ms 变化值和原始 PWM 计数，集中在 `motor_config.h`。方向测试不启用闭环反馈保护，但严格到时停止且每次上电只运行一次。

## 7. 中断名称

- 左编码器 GPIOB：`GROUP1_IRQHandler`，PB0/PB1 任一边沿触发软件正交解码。
- UART0：`UART0_IRQHandler`（源码通过 `DEBUG_UART_INST_IRQHandler` 生成宏声明）。
- 100 Hz 控制定时器：`TIMG0_IRQHandler`（源码通过 `CONTROL_TIMER_INST_IRQHandler` 生成宏声明）。

本工程只完成源代码、SysConfig 和构建级验证，不代表电机、电源、编码器电平、CPR、方向或机械负载已经在实物上验证。
