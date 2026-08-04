# Debug Records
![alt text](images\bps\AMR-1.jpg)  
![alt text](images\bps\AMR-2.jpg)
![alt text](images\bps\AMR-3.jpg)
## 一、波特率导致的双轮启停打滑问题
### 1、115200bps
![alt text](images\bps\LD2-RS-115200-1.png)
![images/bps/5b1125c759ad388860079fb3b4037e10.png](images\bps\LD2-RS-115200bps-2.png)  
### 2、57600bps
![images/bps/cb87409b5c3a6940ce801bc9867ce8e5.png19200bps](images\bps\LD2-RS-57600bps.png) 
### 3、19200bps 
![images/bps/09a5969ec47c39f890af26f437a93b63.png](images\bps\LD2-RS-19200bps.png)  
> 可见，当波特率提升，i.e.帧发送频率提升，双轮抢占总线的间隔缩短，达到拟同时驱动的效果，缓解了运行的非同时性  

## 二、离线状态下 m1_cycle_ms / m2_cycle_ms ≈ 3664ms 跳变分析

### 现象

两台电机均离线（断线/断电/站号错）时，VOFA CH16/CH17 显示的 `m1_cycle_ms` 和 `m2_cycle_ms` 在 **~3664ms** 附近跳变。

### 根源：两电机在单条 RS485 总线上互相交错超时

调度器按"单次 Modbus 事务"粒度轮询：M1 READ → M2 READ → M1 WRITE → M2 WRITE。
每台离线电机的单次事务走完 BSP 3 次物理重试才报 TIMEOUT，然后 FSM 层再用 3 次逻辑重试（每次逻辑重试发 `ERROR_RELEASED` 触发 advance，给另一电机插入机会），两电机完全交错。

### 逐层超时参数

| 层级 | 宏 | 值 | 说明 |
|------|-----|-----|------|
| 事务超时 | `APP_LD2_MOTOR_TIMEOUT_MS` | 100ms | `>` 判断，实测触发在 101~105ms |
| BSP 物理重试 | `BSP_RS485_MAX_RETRY` | 3 | 每事务最多 3 次物理重发 |
| FSM 逻辑重试 | `APP_LD2RS_TASK_MAX_RETRY` | 3 | 读/写各最多 3 次逻辑重试 |

### 完整交错时序

```
Slot  电机  事务         BSP重试×3     ≈耗时      后动作
─────────────────────────────────────────────────────
 1    M1    READ  att1  3×~102ms     ≈306ms     advance
 2    M2    READ  att1  3×~102ms     ≈306ms     advance
 3    M1    READ  att2  3×~102ms     ≈306ms     advance
 4    M2    READ  att2  3×~102ms     ≈306ms     advance
 5    M1    READ  att3  3×~102ms     ≈306ms     耗尽→强制零速, advance
 6    M2    READ  att3  3×~102ms     ≈306ms     耗尽→强制零速, advance
 7    M1    WRITE att1  3×~102ms     ≈306ms     advance
 8    M2    WRITE att1  3×~102ms     ≈306ms     advance
 9    M1    WRITE att2  3×~102ms     ≈306ms     advance
10    M2    WRITE att2  3×~102ms     ≈306ms     advance
11    M1    WRITE att3  3×~102ms     ≈306ms     耗尽, advance
12    M2    WRITE att3  3×~102ms     ≈306ms     耗尽, advance
13    M1    WRITE_DONE                          cycle_elapsed_ms 计算

≈ 12 × 306ms = 3672ms (+主循环 RC/VOFA/LED 开销 → ~3664ms)
```

### 为什么用 `>` 而非 `>=` 导致跳变

```c
// bsp_rs485.c — 超时判断
if ((now_tick - bus->start_tick_ms) > bus->timeout_ms)  // 严格大于 100ms
```

`>` 配合主循环轮询粒度不固定（RC 处理、VOFA 发送等），每次超时的实际延迟在 **101~105ms** 之间波动。12 次事务累积后，cycle_ms 在 **3640~3680ms** 之间跳变。

### 关键代码路径

- 超时判断：[bsp_rs485.c:172-178](BSP/Src/bsp_rs485.c#L172-L178)
- FSM 逻辑重试触发切换：[app_ld2rs_task.c:391-404](App/Src/app_ld2rs_task.c#L391-L404)
- cycle_ms 计算：[app_ld2rs_task.c:521-526](App/Src/app_ld2rs_task.c#L521-L526)

### 结论

3664ms 不是 Bug，是两台离线电机在一条 RS485 总线上按"单次事务轮询"调度器互相交错超时的正常耗时。若未来电机数 N 增加，离线 cycle_ms ≈ N × 12 × 102ms。

---

## 三、CMSIS-DSP 本地化记录

### 现象

原先 Keil RTE 勾选 `CMSIS -> DSP` 后，DSP 头文件和源码可能来自本机 Keil Pack 路径。工程移交给其他电脑时，如果对方没有相同 Pack 或路径不同，容易出现头文件或源码找不到的问题。

### 当前处理

- 将 CMSIS-DSP 放入工程根目录 `DSP/`。
- Keil 工程引用 `..\DSP\Include`、`..\DSP\PrivateInclude` 和 `../DSP/Source/...`。
- Makefile 引用 `DSP/Include`、`DSP/PrivateInclude`，并编译 `DSP/Source` 下非 F16 聚合源文件。
- Makefile 定义 `DISABLEFLOAT16`，当前 Cortex-M7/float32 使用场景不依赖 F16 路径。
- Keil RTE 的 `CMSIS:DSP Source` 不应再重复启用。

### 验证结论

无 `.lib` 也能链接成功是正常的：`arm_math.h` 负责声明，函数实现由 `DSP/Source` 下的 `.c` 编译后提供。`arm_sin_f32()` 依赖 `FastMathFunctions.c` 中的实现和 `CommonTables.c` 中的查表数据。
