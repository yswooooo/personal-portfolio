# VOFA DWT 编码器估算转速通道设计

## 目标

将两台 LD2-RS 电机基于相邻有效 `PrB.24` 位置反馈和 DWT 时间差得到的电机侧估算转速发送到 VOFA，便于与 `PrB.06` 未滤波转速直接比较。

## 数据发布

沿用全局 `extern vofa_motor_info_t g_vofa_speed`，在 `vofa_motor_info_t` 中增加：

- `m1_encoder_estimated_speed_rpm`
- `m2_encoder_estimated_speed_rpm`

`app_ld2rs_task_run()` 完成 M1/M2 编码器速度估算更新后，通过一个接收 `vofa_motor_info_t *` 的发布函数，把两个估算器的 `motor_speed_rpm` 复制到上述字段。主循环只读取 `g_vofa_speed`，不直接访问私有 FSM 或估算器。

## VOFA 通道

JustFloat 数组由 19 路扩展到 21 路，原有通道顺序保持不变：

| 通道 | 数据 | 对比通道 |
|------|------|----------|
| CH20 | M1 DWT 编码器估算电机转速，rpm | CH2 `PrB.06` |
| CH21 | M2 DWT 编码器估算电机转速，rpm | CH4 `PrB.06` |

## 边界行为

- 首个有效编码器样本只建立差分基准，估算转速保持初始化值 0。
- CRC 错误、超时或其他无效反馈不发布新编码器样本，因此不会生成新的估算转速。
- 本次只发送电机侧 rpm，不增加轮毂 rpm 和轮毂线速度通道。

## 验证

- 单元测试确认 M1/M2 估算值被发布到各自的 VOFA 字段，互不串扰。
- 静态检查确认 JustFloat 数组和发送数量均为 21，CH20/CH21 顺序正确。
- 运行编码器相关测试并完成 GCC 固件构建。
