# LD2-RS 基于 DWT 的编码器速度估算设计

## 目标

在当前 STM32H723 裸机工程中，基于相邻两次有效 `PrB.24`
编码器位置反馈和对应的 DWT 接收时间戳，分别估算 M1、M2
的区间平均速度，同时修复 BSP RS485 中断与主循环共享控制字段缺少
`volatile` 的问题。

本次修改保持现有 RS485 FSM、Modbus 事务、双电机轮询顺序、超时重试、
离线策略、目标速度写入、VOFA 发送方式和 `WRITE_REQ` 启动失败处理不变。
工程没有已配置的合适 TIM，本次不新增或配置 TIM。

## 已确认的设备参数

`Pr0.08 = 10000` 表示电机每转一圈的输入指令脉冲数，属于电子齿轮的
指令单位，不是 `PrB.24` 的编码器反馈单位，因此不参与本次反馈速度换算。

电机铭牌后缀为 `M17`。根据手册第 14 页的电机型号规则，`M` 表示磁电
编码器，`17` 表示 17 位分辨率，第三字段缺省表示多圈编码器。因此当前
电机的编码器分辨率已确认为 17 位，每机械圈为 131072 个编码器计数：

```c
#define APP_LD2_ENCODER_BITS  17U
```

配置只允许手册明确列出的 17 位和 23 位：

```c
#if APP_LD2_ENCODER_BITS == 17U
#define LD2_ENCODER_COUNTS_PER_REV  (131072.0f)
#elif APP_LD2_ENCODER_BITS == 23U
#define LD2_ENCODER_COUNTS_PER_REV  (8388608.0f)
#else
#error "APP_LD2_ENCODER_BITS must be 17U or 23U"
#endif
```

车轮侧换算复用工程已有参数，不重复定义：

```c
#define APP_CHASSIS_WHEEL_RADIUS_MM  (75.0f)
#define APP_CHASSIS_GEAR_RATIO       (20.0f)
```

减速比物理含义为“电机转速 : 车轮转速 = 20 : 1”。M17 编码器位于电机
侧，因此 `PrB.24` 每变化 131072 个编码器单位代表电机轴一圈，而车轮
一圈对应 2621440 个编码器单位。

将来确认使用 23 位编码器时，只需把 `APP_LD2_ENCODER_BITS` 改为
`23U`。`PrB.24` 的数据格式为 32 位累计位置，这不表示编码器为 32 位，
因此不允许把该宏配置为 `32U`。

## 方案选择

采用独立、纯计算的编码器速度估算模块。该方案比把所有逻辑继续堆入
`app_ld2rs_task.c` 多两个小文件，但能让样本类型、估算状态和计算函数
保持职责单一，并可由主机 C 测试直接验证。直接在 FSM 内写静态计算函数
虽然文件更少，但会进一步加重现有任务文件的耦合。直接使用驱动器速度
寄存器则不满足“相邻有效位置样本加 DWT 时间差”的目标。

## 组件与接口

### 编码器速度估算模块

新增 `App/Inc/app_encoder_speed.h` 和 `App/Src/app_encoder_speed.c`。
模块只依赖标准整数类型，不依赖 HAL、RS485 或具体电机对象。

编码器样本：

```c
typedef struct
{
    int32_t counts;
    uint64_t timestamp_us;
    uint32_t sequence;
} app_encoder_sample_t;
```

每台电机独立的估算状态：

```c
typedef struct
{
    uint8_t initialized;
    uint8_t speed_valid;
    uint8_t is_stale;
    int32_t last_counts;
    uint64_t last_timestamp_us;
    uint32_t last_sequence;
    int64_t delta_counts;
    uint64_t delta_time_us;
    float speed_counts_per_s;
    float motor_speed_rpm;
    float wheel_speed_rpm;
    float wheel_speed_mps;
} app_encoder_speed_estimator_t;
```

计算接口：

```c
void app_encoder_speed_update(
    const app_encoder_sample_t *sample,
    app_encoder_speed_estimator_t *estimator,
    int8_t encoder_polarity);
```

同一 `sequence` 立即返回。第一次样本只建立基线；第二次及之后的新样本
才计算速度。`delta_time_us == 0` 时不除零，但仍消费该样本并更新基线，
避免后续反复处理同一个异常样本。

速度定义为：

```text
raw_delta_counts = 当前位置 - 上次位置
delta_counts     = encoder_polarity × raw_delta_counts
counts/s         = delta_counts × 1,000,000 / delta_time_us
motor rpm        = counts/s × 60 / LD2_ENCODER_COUNTS_PER_REV
wheel rpm        = motor rpm / APP_CHASSIS_GEAR_RATIO
wheel m/s        = wheel rpm × 2π × (APP_CHASSIS_WHEEL_RADIUS_MM / 1000) / 60
```

M1、M2 使用与控制指令极性相互独立的配置：

```c
#define APP_LD2_ENCODER_POLARITY_M1  1
#define APP_LD2_ENCODER_POLARITY_M2  (-1)
```

初始值与当前两侧电机的安装方向一致，但不引用或别名到
`APP_MOTOR_CTRL_M1_POLARITY`、`APP_MOTOR_CTRL_M2_POLARITY`。如果实机
验证发现编码器递增方向与期望机械正方向相反，只修改对应编码器极性宏。
核心公式始终使用 `now_counts - last_counts`，不通过交换两者隐藏负号。

### DWT 微秒时间

复用现有 `bsp_dwt`，新增 `uint64_t BSP_DWT_GetTickUs(void)`，不创建第二套
DWT 驱动。毫秒和微秒接口共享同一个受 PRIMASK 临界区保护的 64 位周期
累加更新函数，避免两个接口各自更新同一累计状态。

480 MHz 下 32 位 `CYCCNT` 约 8.95 秒回绕一次。软件扩展能安全处理相邻
调用间的一次回绕；主循环任务会持续调用 DWT 接口维护累计值，因此正常
运行和通信离线期间不会因仅在 RX 中断调用而漏掉多次回绕。
具体做法是在每次 `app_ld2rs_task_run()` 开始时调用一次
`BSP_DWT_GetTickUs()`；该值同时作为未来过期检测的当前时间。这个调用
不调度事务，也不改变 FSM 状态。

### BSP RS485 发布

在 `bsp_rs485_handle_t` 中将真实由 ISR 写、主循环读的标量声明为
`volatile`：

```c
volatile bsp_rs485_state_t state;
volatile uint16_t rx_len;
volatile uint64_t rx_timestamp_us;
volatile uint8_t rx_error_code;
volatile uint8_t state_error_code;
```

不把整个句柄或 RX/TX 缓冲区声明为 `volatile`。

`bsp_rs485_rx_event_callback()` 在确认当前状态为 `RX_WAIT` 后，严格按照
以下顺序发布：

```c
bus->rx_len = rx_size;
bus->rx_timestamp_us = BSP_DWT_GetTickUs();
bus->state = BSP_RS485_STATE_DONE;
```

主循环观察到 `DONE` 时，长度和时间戳已经写完。启动、确认、取消和重试
新事务时清理旧 `rx_len` 与旧时间戳，避免错误复用。

### APP FSM 集成

`motor_fsm_t` 为每台电机分别持有一个 `app_encoder_sample_t` 和一个
`app_encoder_speed_estimator_t`。只有 `APP_LD2RS_TASK_PHASE_WAIT_READ`
中的编码器响应通过现有长度、CRC、站号、功能码和字节数校验，并成功把
高低寄存器拼成 `int32_t` 后，先构造完整的新样本，再把源结构体和目标
结构体的地址传给简单发布函数：

```c
app_encoder_sample_t new_sample = {
    .counts = parsed_encoder_counts,
    .timestamp_us = bus->rx_timestamp_us,
    .sequence = fsm->encoder_sample.sequence + 1u
};

app_encoder_sample_publish(
    &fsm->encoder_sample,
    &new_sample);
```

发布函数只按 `counts`、`timestamp_us`、`sequence` 的顺序复制字段，
`sequence` 最后写入。它不接收分散的标量赋值形参，也不使用无法保证发布
顺序的整体结构体赋值。

CRC 错误、长度错误、站号或功能码错误、BSP `TIMEOUT/ERROR`、重试耗尽
均不增加 `sequence`，也不会把保留的旧位置重新发布为新样本。

速度计算在 `app_ld2rs_task_run()` 的主循环上下文中执行。M1、M2 每次都
调用更新函数；无新序号时函数立即返回。USART 中断只记录接收元数据，
不执行浮点除法、滤波或业务计算。

为外部控制或调试提供只读结果类型和查询接口：

```c
typedef struct
{
    float motor_speed_rpm;
    float wheel_speed_rpm;
    float wheel_speed_mps;
} app_ld2rs_speed_feedback_t;

bool app_ld2rs_task_get_speed_feedback(
    uint8_t motor_number,
    app_ld2rs_speed_feedback_t *feedback);
```

`motor_number` 只接受 `1U` 或 `2U`，输出指针必须非空；参数无效或尚未
形成有效速度时返回 `false`，否则复制最近一次的电机 RPM、车轮 RPM 和
车轮线速度并返回 `true`。`speed_counts_per_s` 保留在估算器中供调试，
不作为本次公开结果之一。查询接口不改变现有 VOFA 数据结构和发送通道。

## 32 位累计位置回绕

`PrB.24` 是两个 16 位寄存器拼接出的 32 位累计位置。位置差采用 32 位
无符号模运算，再显式转换成 `int64_t` 的最短环形差：

```c
uint32_t wrapped_delta =
    (uint32_t)now_counts - (uint32_t)last_counts;

int64_t delta_counts =
    (wrapped_delta <= (uint32_t)INT32_MAX)
        ? (int64_t)wrapped_delta
        : (int64_t)wrapped_delta - (1LL << 32);
```

随后在 `int64_t` 上应用 `encoder_polarity`。该方法避免两个 `int32_t`
直接相减溢出，也避免把超出 `INT32_MAX` 的 `uint32_t` 直接转换为
`int32_t` 的实现相关行为。它能处理
`INT32_MAX -> INT32_MIN` 及反向跨界，不使用错误的 16 位单圈阈值逻辑。
它要求相邻有效样本间的真实位移绝对值小于 `2^31` 个编码器计数；在当前
轮询周期和速度上限下有充分余量。

## 过期反馈

估算状态预留 `is_stale`。由于当前没有经过确认的过期微秒阈值，本次不
据此清零速度、停机或改变任何安全策略。通信失败不会伪造零速，也不会
发布新样本。后续确定正常六帧轮询周期和控制需求后，再配置过期阈值并
启用状态更新。

## 错误处理

- 空样本或空估算器指针：立即返回。
- 无新序号：立即返回。
- 第一次有效样本：只保存基线，速度无效。
- 时间差为零：不计算速度，速度无效，更新基线。
- 无效 Modbus 响应或通信超时：不发布样本，保留最近一次估算结果。
- DWT 未初始化：时间戳为零；首样本仍只建基线，重复零时间戳不会除零。

## 测试与验证

新增纯计算单元测试并扩展现有 BSP/APP 主机测试，覆盖：

1. 第一次样本只初始化；
2. 第二次样本得到正确位置差、时间差、`counts/s` 和 17 位电机 RPM；
3. 编码器递增、递减和 `encoder_polarity = -1`；
4. `delta_time_us == 0` 不除零；
5. 相同 `sequence` 不重复计算；
6. 32 位累计位置正向、反向跨界；
7. `motor rpm / 20` 得到正确车轮 RPM；
8. 结合 75 mm 半径得到正确车轮线速度；
9. CRC 或格式无效时不增加样本序号；
10. `TIMEOUT/ERROR` 不发布旧位置为新样本；
11. M1、M2 样本、极性和估算状态互不干扰；
12. RX 回调按长度、时间戳、`DONE` 顺序发布；
13. ISR/主循环共享字段的 `volatile` 声明可通过编译检查；
14. 原有 FSM 状态数、轮询顺序和 Modbus 行为保持不变；
15. 主机测试使用 `-Wall -Wextra -Werror`，无新增警告；
16. 可用构建环境存在时执行 STM32 工程编译。

当前工作树已有与本任务无关的用户修改，实施时仅做范围内的最小补丁，
不覆盖或回退这些修改。现有测试中的基线失败将与本功能测试结果分开
记录，不借本任务修改无关 FSM 分支。
