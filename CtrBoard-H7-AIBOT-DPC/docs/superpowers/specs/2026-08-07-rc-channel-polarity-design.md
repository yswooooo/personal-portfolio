# RC 摇杆通道极性统一设计

## 1. 目标

在 SBUS 有效帧转换为 `g_rc` 命名通道时统一摇杆方向，使四个摇杆轴满足：

- `ch_rx`、`ch_lx`：向上为正；
- `ch_ry`、`ch_ly`：向左为正。

调整后保持现有实车行为不变，即右摇杆左打仍产生正角速度并使差速底盘左转。

## 2. 修改范围

1. 在 `BSP/Inc/bsp_rc.h` 为四个摇杆通道增加独立极性宏：
   `BSP_RC_CH_RY_POLARITY`、`BSP_RC_CH_RX_POLARITY`、
   `BSP_RC_CH_LX_POLARITY`、`BSP_RC_CH_LY_POLARITY`。
2. 在 `BSP/Src/bsp_rc.c::bsp_rc_on_frame_received()` 中，将每个通道的
   “SBUS 原始值减中位值”结果乘以对应极性宏。
3. 将 `APP_CHASSIS_CMD_POLARITY` 从 `-1` 调整为 `1`，取消应用层对
   已统一方向的 `ch_ry` 再次反号。
4. 同步更新通道方向和角速度极性宏的注释。

不修改低通滤波器、死区判断、差速运动学公式、M1/M2 电机安装极性、
RS485 状态机和电机寄存器写入逻辑。

## 3. 极性定义

```c
#define BSP_RC_CH_RY_POLARITY    (-1)
#define BSP_RC_CH_RX_POLARITY    ( 1)
#define BSP_RC_CH_LX_POLARITY    ( 1)
#define BSP_RC_CH_LY_POLARITY    (-1)
```

`APP_CHASSIS_CMD_POLARITY` 调整为：

```c
#define APP_CHASSIS_CMD_POLARITY (1)
```

## 4. 数据流

```text
SBUS raw channel
  -> 减去 BSP_SBUS_MID_VALUE
  -> 乘以 BSP_RC_CH_*_POLARITY
  -> g_rc.ch_*
  -> bsp_rc_filter_update()
  -> g_rc_filter.ch_*
  -> ch_rx 映射为线速度、ch_ry 映射为角速度
  -> 差速解算
  -> M1/M2 安装极性
  -> Pr3.04 目标转速
```

滤波器直接使用 `g_rc` 作为输入，因此不在滤波函数中重复处理极性。
`ch_ly` 当前只更新和滤波，尚未参与底盘控制。

## 5. 期望符号

| 遥控动作 | 命名通道 | 映射结果 | 实车行为 |
| --- | --- | --- | --- |
| 右摇杆上打 | `ch_rx > 0` | `v > 0` | 前进 |
| 右摇杆下打 | `ch_rx < 0` | `v < 0` | 后退 |
| 右摇杆左打 | `ch_ry > 0` | `omega > 0` | 左转 |
| 右摇杆右打 | `ch_ry < 0` | `omega < 0` | 右转 |
| 左摇杆上打 | `ch_lx > 0` | 当前未参与底盘控制 | 无变化 |
| 左摇杆左打 | `ch_ly > 0` | 当前未参与底盘控制 | 无变化 |

## 6. 验证

1. 使用 GCC `make` 验证工程编译和链接。
2. Keil5 在线观察 `g_rc` 与 `g_rc_filter`，确认上打为正、左打为正。
3. 观察 `g_rc_chassis.fLinearVel`：右摇杆上打时为正。
4. 观察 `g_rc_chassis.fAngularVel`：右摇杆左打时为正。
5. 在低速和车轮悬空条件下确认前进、后退、左转、右转方向未发生变化。

## 7. 风险控制

- 极性只在 RC 数据进入命名通道的边界处理一次，避免重复反号。
- `APP_CHASSIS_CMD_POLARITY` 必须与 `ch_ry` 的入口极性成对调整，否则实车转向会反转。
- 不调整 `APP_MOTOR_CTRL_M1_POLARITY` 和 `APP_MOTOR_CTRL_M2_POLARITY`，它们只负责补偿电机镜像安装方向。
