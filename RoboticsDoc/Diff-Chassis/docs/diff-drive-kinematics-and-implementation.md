# 双轮差速底盘运动学推导与工程代码实现
---
![双轮差速底盘右手坐标系运动学模型](/assets/diff-drive/Fig.1%20diff-drive-right-hand-coordinate-system.jpg)
<p align="center">
  <em>Fig.1 双轮差速底盘右手坐标系运动学模型</em>
</p>

---
## 1. 机器人右手坐标系与符号约定
| 符号                     | 含义                                    |               单位 | 类型   |
| ---------------------- | ------------------------------------- | ---------------: | ---- |
| $V_{\mathrm{robot}}$ | 机体中心期望线速度                             |   $\mathrm{m/s}$ | 输入   |
| $W_{\mathrm{robot}}$ | 机体期望偏航角速度                             | $\mathrm{rad/s}$ | 输入   |
| $v_{\mathrm{left}}$    | 左轮期望线速度                               |   $\mathrm{m/s}$ | 输出   |
| $v_{\mathrm{right}}$   | 右轮期望线速度                               |   $\mathrm{m/s}$ | 输出   |
| $H$                    | 左右轮毂中心间距                              |     $\mathrm{m}$ | 结构参数 |
| $R$                    | 机体中心到瞬时旋转中心 $O_{\mathrm{Motion}}$ 的半径 |     $\mathrm{m}$ | 中间量  |
| $R_{\mathrm{Left}}$    | 左轮到瞬时旋转中心的半径                          |     $\mathrm{m}$ | 中间量  |
| $R_{\mathrm{Right}}$   | 右轮到瞬时旋转中心的半径                          |     $\mathrm{m}$ | 中间量  |

---
## 2. 双轮差速底盘运动学数学推导

### 2.1 机体中心线速度与偏航角速度的关系

双轮差速底盘绕瞬时旋转中心 $O_{\mathrm{Motion}}$ 做圆周运动时，机体中心期望线速度、机体期望偏航角速度和转弯半径之间满足：

$$
V_{\mathrm{robot}}
=
R * W_{\mathrm{robot}}
\tag{1}
$$

其中：

- $V_{\mathrm{robot}}$ 为机体中心期望线速度，单位为 $\mathrm{m/s}$；
- $W_{\mathrm{robot}}$ 为机体期望偏航角速度，单位为 $\mathrm{rad/s}$；
- $R$ 为机体中心到瞬时旋转中心 $O_{\mathrm{Motion}}$ 的带符号半径，单位为 $\mathrm{m}$。

在标准右手坐标系下：

- $W_{\mathrm{robot}}>0$ 表示底盘逆时针旋转；
- $W_{\mathrm{robot}}<0$ 表示底盘顺时针旋转。

### 2.2 左右轮转弯半径

左右轮毂中心间距为 $H$，因此左右轮相对机体中心的横向距离均为：

$$
\frac{H}{2}
$$

左右轮到瞬时旋转中心的带符号半径分别为：

$$
\begin{aligned}
R_{\mathrm{Left}}
&=
R-\frac{H}{2}
\\
R_{\mathrm{Right}}
&=
R+\frac{H}{2}
\end{aligned}
\tag{2}
$$

因此，左右轮期望线速度分别为：

$$
\begin{aligned}
v_{\mathrm{left}}
&=
R_{\mathrm{Left}}*W_{\mathrm{robot}}
\\
&=
\left(
R-\frac{H}{2}
\right)*W_{\mathrm{robot}}
\\[6pt]
v_{\mathrm{right}}
&=
R_{\mathrm{Right}}*W_{\mathrm{robot}}
\\
&=
\left(
R+\frac{H}{2}
\right)*W_{\mathrm{robot}}
\end{aligned}
\tag{3}
$$

### 2.3 消去虚拟转弯半径

$R$ 是运动学推导中引入的中间变量，并不是底盘控制器的直接期望输入。

当：

$$
W_{\mathrm{robot}}\neq0
$$

由式（1）可得：

$$
R
=
\frac{V_{\mathrm{robot}}}
{W_{\mathrm{robot}}}
\tag{4}
$$

将式（4）代入左轮期望线速度公式：

$$
\begin{aligned}
v_{\mathrm{left}}
&=
\left(
\frac{V_{\mathrm{robot}}}
{W_{\mathrm{robot}}}
-\frac{H}{2}
\right)*
W_{\mathrm{robot}}
\\
&=
V_{\mathrm{robot}}
-\frac{H}{2}W_{\mathrm{robot}}
\end{aligned}
\tag{5}
$$

将式（4）代入右轮期望线速度公式：

$$
\begin{aligned}
v_{\mathrm{right}}
&=
\left(
\frac{V_{\mathrm{robot}}}
{W_{\mathrm{robot}}}
+\frac{H}{2}
\right)*
W_{\mathrm{robot}}
\\
&=
V_{\mathrm{robot}}
+\frac{H}{2}W_{\mathrm{robot}}
\end{aligned}
\tag{6}
$$

需要注意，展开过程中 $\dfrac{H}{2}$ 必须继续乘以 $W_{\mathrm{robot}}$，否则公式两侧的物理量单位不一致：

$$
\frac{H}{2}*W_{\mathrm{robot}}
,\quad
\left[
\mathrm{m}\cdot\frac{\mathrm{rad}}{\mathrm{s}}
\right]
=
\left[
\mathrm{m/s}
\right]
$$

其中弧度 $\mathrm{rad}$ 为无量纲单位。

### 2.4 直线运动的特殊情况

当底盘做直线运动时：

$$
W_{\mathrm{robot}}=0
$$

此时不能通过：

$$
R
=
\frac{V_{\mathrm{robot}}}
{W_{\mathrm{robot}}}
$$

计算转弯半径，但消去 $R$ 后的左右轮速度公式仍然成立：

$$
\begin{aligned}
v_{\mathrm{left}}
&=
V_{\mathrm{robot}}
-\frac{H}{2}\times0
=
V_{\mathrm{robot}}
\\
v_{\mathrm{right}}
&=
V_{\mathrm{robot}}
+\frac{H}{2}\times0
=
V_{\mathrm{robot}}
\end{aligned}
$$

因此：

$$
v_{\mathrm{left}}
=
v_{\mathrm{right}}
=
V_{\mathrm{robot}}
\tag{7}
$$

在代码实现中不需要实际计算虚拟转弯半径 $R$，从而避免 $W_{\mathrm{robot}}=0$ 时出现除零问题。

### 2.5 双轮差速底盘逆运动学方程

将式（5）和式（6）写成方程组：

$$
\begin{cases}
v_{\mathrm{left}}
=
V_{\mathrm{robot}}
-\dfrac{H}{2}W_{\mathrm{robot}}
\\[6pt]
v_{\mathrm{right}}
=
V_{\mathrm{robot}}
+\dfrac{H}{2}W_{\mathrm{robot}}
\end{cases}
\tag{8}
$$

写成矩阵形式：

$$
\begin{bmatrix}
v_{\mathrm{left}}
\\[9pt]
v_{\mathrm{right}}
\end{bmatrix}
=
\begin{bmatrix}
1 & -\dfrac{H}{2}
\\[9pt]
1 & \dfrac{H}{2}
\end{bmatrix}
\begin{bmatrix}
V_{\mathrm{robot}}
\\[9pt]
W_{\mathrm{robot}}
\end{bmatrix}
\tag{9}
$$

令运动学变换矩阵为：


$$
\mathbf{A}
=
\begin{bmatrix}
1 & -\dfrac{H}{2}
\\[9pt]
1 & \dfrac{H}{2}
\end{bmatrix}
\tag{10}
$$

令左右轮期望线速度向量为：

$$
\mathbf{V}_{\mathrm{wheel}}
=
\begin{bmatrix}
v_{\mathrm{left}}
\\[6pt]
v_{\mathrm{right}}
\end{bmatrix}
\tag{11}
$$

令机体期望速度向量为：

$$
\mathbf{V}_{\mathrm{robot}}
=
\begin{bmatrix}
V_{\mathrm{robot}}
\\[6pt]
W_{\mathrm{robot}}
\end{bmatrix}
\tag{12}
$$

则双轮差速底盘逆运动学模型可简写为：

$$
\mathbf{V}_{\mathrm{wheel}}
=
\mathbf{A}
\mathbf{V}_{\mathrm{robot}}
\tag{13}
$$
## 3.双轮差速底盘运动学到代码实现

### 3.1 运动学计算过程

由式（8）可得知，双轮差速底盘逆运动学公式为：

$$
\begin{cases}
v_{\mathrm{left}}
=
V_{\mathrm{robot}}
-\dfrac{H}{2}W_{\mathrm{robot}}
\\[6pt]
v_{\mathrm{right}}
=
V_{\mathrm{robot}}
+\dfrac{H}{2}W_{\mathrm{robot}}
\end{cases}
\tag{14}
$$

其中，左右轮线速度单位均为 $\mathrm{m/s}$。

设轮毂半径为 $r$，轮毂角速度为：

$$
\begin{cases}
\omega_{\mathrm{wheel,left}}
=
\dfrac{v_{\mathrm{left}}}{r}
\\[9pt]
\omega_{\mathrm{wheel,right}}
=
\dfrac{v_{\mathrm{right}}}{r}
\end{cases}
\tag{15}
$$

轮毂角速度单位为 $\mathrm{rad/s}$。

轮毂转速为：

$$
\begin{cases}
n_{\mathrm{wheel,left}}
=
\dfrac{60v_{\mathrm{left}}}{2\pi r}
\\[9pt]
n_{\mathrm{wheel,right}}
=
\dfrac{60v_{\mathrm{right}}}{2\pi r}
\end{cases}
\tag{16}
$$

定义减速比：

$$
i
=
\frac{n_{\mathrm{motor}}}
{n_{\mathrm{wheel}}}
\tag{17}
$$

其中，$i>1$ 表示电机转子转速高于轮毂转速。

因此，左右电机转子目标转速为：

$$
\begin{cases}
n_{\mathrm{motor,left}}
=
\dfrac{60v_{\mathrm{left}}i}{2\pi r}
\\[9pt]
n_{\mathrm{motor,right}}
=
\dfrac{60v_{\mathrm{right}}i}{2\pi r}
\end{cases}
\tag{18}
$$

将式（14）代入式（18），得到：

$$
\begin{cases}
n_{\mathrm{motor,left}}
=
\left(
V_{\mathrm{robot}}
-\dfrac{H}{2}W_{\mathrm{robot}}
\right)
\dfrac{60i}{2\pi r}
\\[11pt]
n_{\mathrm{motor,right}}
=
\left(
V_{\mathrm{robot}}
+\dfrac{H}{2}W_{\mathrm{robot}}
\right)
\dfrac{60i}{2\pi r}
\end{cases}
\tag{19}
$$

### 3.2 机械参数宏定义

理论符号 $H$ 在代码中使用带单位的宏名称
`CHASSIS_DIFF_WHEEL_TRACK_M` 表示，以避免单字母宏与其他代码发生冲突。

```c
/* 以下数值仅为示例，必须替换为底盘的实际机械参数。 */

/* H：左右轮毂中心间距，单位为 m。 */
#define CHASSIS_DIFF_WHEEL_TRACK_M       (0.500f)

/* r：轮毂半径，单位为 m。 */
#define CHASSIS_DIFF_WHEEL_RADIUS_M      (0.100f)

/*
 * i：减速比。
 * 定义为：电机转子转速 / 轮毂转速。
 */
#define CHASSIS_DIFF_MOTOR_GEAR_RATIO    (2.000f)

/* 2π，用于将轮毂线速度转换为转速。 */
#define CHASSIS_DIFF_TWO_PI_F            (6.28318530717958647692f)
```

### 3.3 轮毂线速度到电机转子转速

```c
/**
 * @brief 将轮毂线速度转换为电机转子转速。
 *
 * @param wheel_speed_mps 轮毂线速度，单位为 m/s。
 *
 * @return 电机转子转速，单位为 rpm。
 */
static float chassis_diff_wheel_mps_to_motor_rpm(
    float wheel_speed_mps)
{
    return wheel_speed_mps
           * 60.0f
           * CHASSIS_DIFF_MOTOR_GEAR_RATIO
           / (CHASSIS_DIFF_TWO_PI_F
              * CHASSIS_DIFF_WHEEL_RADIUS_M);
}
```

输入的轮毂线速度保留正负号，因此转换得到的电机转速也保留正负号：

- 正 RPM：轮子驱动底盘向前；
- 负 RPM：轮子驱动底盘向后。

电机实际安装方向引起的正负极性差异，应在电机驱动层中处理。

### 3.4 双轮差速底盘逆运动学函数

```c
#include <stddef.h>

/**
 * @brief 将机体期望速度转换为左右电机转子目标转速。
 *
 * 标准右手坐标系约定：
 * 1. V_robot > 0 表示底盘向前运动；
 * 2. W_robot > 0 表示底盘逆时针旋转；
 * 3. 输出转速保留正负方向。
 *
 * @param v_robot  机体中心期望线速度，单位为 m/s。
 * @param w_robot  机体期望偏航角速度，单位为 rad/s。
 * @param rpm_left 左电机转子目标转速输出，单位为 rpm。
 * @param rpm_right 右电机转子目标转速输出，单位为 rpm。
 */
void chassis_diff_cmd(float v_robot,
                      float w_robot,
                      float *rpm_left,
                      float *rpm_right)
{
    float v_left;
    float v_right;

    if ((rpm_left == NULL) || (rpm_right == NULL)) {
        return;
    }

    /* 双轮差速底盘逆运动学。 */
    v_left = v_robot - 0.5f * CHASSIS_DIFF_WHEEL_TRACK_M * w_robot;

    v_right = v_robot + 0.5f * CHASSIS_DIFF_WHEEL_TRACK_M * w_robot;

    /* 轮毂线速度转换为电机转子转速。 */
    *rpm_left = chassis_diff_wheel_mps_to_motor_rpm(v_left);

    *rpm_right =chassis_diff_wheel_mps_to_motor_rpm(v_right);
}
```

### 3.5 函数调用示例

```c
float left_motor_target_rpm;
float right_motor_target_rpm;

chassis_diff_cmd(
    1.0f,                   /* V_robot：1.0 m/s 这里可以替换为上位机给STM32发送的控制命令*/
    0.5f,                   /* W_robot：0.5 rad/s 这里可以替换为上位机给STM32发送的控制命令*/
    &left_motor_target_rpm,
    &right_motor_target_rpm
);
```

当有：

$$
W_{\mathrm{robot}}>0
$$

表示底盘逆时针旋转，右轮为外侧轮，其转速应大于左轮：

$$
n_{\mathrm{motor,right}}
>
n_{\mathrm{motor,left}}
$$

## 4. 考虑给定量越界：等比例降速保持机体运动姿态

### 4.1 电机最大转速约束

通过双轮差速底盘逆运动学和轮速转换，可以得到左右电机转子期望转速：

$$
n_{\mathrm{motor,left}},
\qquad
n_{\mathrm{motor,right}}
$$

由于电机转子存在最大允许转速，因此左右电机转速需要满足：

$$
\left|n_{\mathrm{motor,left}}\right|
\leq
n_{\mathrm{motor,max}}
$$

$$
\left|n_{\mathrm{motor,right}}\right|
\leq
n_{\mathrm{motor,max}}
$$

其中，$n_{\mathrm{motor,max}}$ 为电机手册规定的最大允许转速，单位为 $\mathrm{rpm}$。

不能分别对左右电机转速进行独立限幅。例如：

$$
n_{\mathrm{motor,left}}=2000\,\mathrm{rpm}
$$

$$
n_{\mathrm{motor,right}}=4000\,\mathrm{rpm}
$$

若电机最大转速为：

$$
n_{\mathrm{motor,max}}=3000\,\mathrm{rpm}
$$

仅将右轮转速截断为 $3000\,\mathrm{rpm}$，会使左右轮原有转速比例发生变化：

$$
\frac{2000}{4000}
\neq
\frac{2000}{3000}
$$

左右轮速度比例改变后，底盘的线速度和偏航角速度比例也会改变，从而改变底盘的运动轨迹曲率。

因此，左右电机必须使用同一个比例系数进行缩放。

### 4.2 找出当前最大电机转速

电机转速包含正负方向，因此比较转速时必须比较其绝对值。

定义当前最大电机转速模为：

$$
n_{\mathrm{current}}
=
\max\left(
\left|n_{\mathrm{motor,left}}\right|,
\left|n_{\mathrm{motor,right}}\right|
\right)
\tag{20}
$$

当：

$$
n_{\mathrm{current}}
\leq
n_{\mathrm{motor,max}}
$$

说明左右电机均未超出最大允许转速，不需要进行缩放。

此时比例系数为：

$$
s=1
\tag{21}
$$

### 4.3 计算等比例降速系数

当：

$$
n_{\mathrm{current}}
>
n_{\mathrm{motor,max}}
$$

定义超速比例：

$$
k
=
\frac{n_{\mathrm{current}}}
{n_{\mathrm{motor,max}}}
\tag{22}
$$

由于当前最大转速已经超过允许值，因此：

$$
k>1
$$

根据超速比例计算等比例缩放系数：

$$
s
=
\frac{1}{k}
=
\frac{n_{\mathrm{motor,max}}}
{n_{\mathrm{current}}}
\tag{23}
$$

因此：

$$
0<s<1
$$

将左右电机原始期望转速同时乘以缩放系数 $s$：

$$
\begin{cases}
n_{\mathrm{motor,left}}'
=
s n_{\mathrm{motor,left}}
\\[6pt]
n_{\mathrm{motor,right}}'
=
s n_{\mathrm{motor,right}}
\end{cases}
\tag{24}
$$

缩放后，原本绝对值最大的电机转速为：

$$
\begin{aligned}
n_{\mathrm{current}}'
&=
s n_{\mathrm{current}}
\\
&=
\frac{n_{\mathrm{motor,max}}}
{n_{\mathrm{current}}}
n_{\mathrm{current}}
\\
&=
n_{\mathrm{motor,max}}
\end{aligned}
\tag{25}
$$

因此，等比例缩放可以保证所有电机转速均不超过最大允许转速。

### 4.4 等比例降速保持左右轮速度比例

缩放前左右电机转速比例为：

$$
\frac{n_{\mathrm{motor,left}}}
{n_{\mathrm{motor,right}}}
$$

等比例缩放后的转速比例为：

$$
\begin{aligned}
\frac{n_{\mathrm{motor,left}}'}
{n_{\mathrm{motor,right}}'}
&=
\frac{s n_{\mathrm{motor,left}}}
{s n_{\mathrm{motor,right}}}
\\
&=
\frac{n_{\mathrm{motor,left}}}
{n_{\mathrm{motor,right}}}
\end{aligned}
\tag{26}
$$

因此，左右电机转速的比例和正负方向均保持不变。

在左右轮半径和减速比相同的前提下，左右轮线速度也使用同一个系数进行缩放：

$$
\begin{cases}
v_{\mathrm{left}}'
=
s v_{\mathrm{left}}
\\[6pt]
v_{\mathrm{right}}'
=
s v_{\mathrm{right}}
\end{cases}
\tag{27}
$$

### 4.5 等比例降速保持机体运动轨迹

双轮差速底盘正运动学公式为：

$$
V_{\mathrm{robot}}
=
\frac{
v_{\mathrm{left}}+v_{\mathrm{right}}
}{2}
\tag{28}
$$

$$
W_{\mathrm{robot}}
=
\frac{
v_{\mathrm{right}}-v_{\mathrm{left}}
}{H}
\tag{29}
$$

等比例缩放后的机体线速度为：

$$
\begin{aligned}
V_{\mathrm{robot}}'
&=
\frac{
v_{\mathrm{left}}'+v_{\mathrm{right}}'
}{2}
\\
&=
\frac{
s v_{\mathrm{left}}+s v_{\mathrm{right}}
}{2}
\\
&=
sV_{\mathrm{robot}}
\end{aligned}
\tag{30}
$$

等比例缩放后的机体偏航角速度为：

$$
\begin{aligned}
W_{\mathrm{robot}}'
&=
\frac{
v_{\mathrm{right}}'-v_{\mathrm{left}}'
}{H}
\\
&=
\frac{
s v_{\mathrm{right}}-s v_{\mathrm{left}}
}{H}
\\
&=
sW_{\mathrm{robot}}
\end{aligned}
\tag{31}
$$

因此，机体线速度和偏航角速度同时乘以相同的缩放系数：

$$
\begin{bmatrix}
V_{\mathrm{robot}}'
\\
W_{\mathrm{robot}}'
\end{bmatrix}
=
s
\begin{bmatrix}
V_{\mathrm{robot}}
\\
W_{\mathrm{robot}}
\end{bmatrix}
\tag{32}
$$

当 $W_{\mathrm{robot}}\neq0$ 时，缩放后的转弯半径为：

$$
\begin{aligned}
R'
&=
\frac{V_{\mathrm{robot}}'}
{W_{\mathrm{robot}}'}
\\
&=
\frac{sV_{\mathrm{robot}}}
{sW_{\mathrm{robot}}}
\\
&=
\frac{V_{\mathrm{robot}}}
{W_{\mathrm{robot}}}
\\
&=
R
\end{aligned}
\tag{33}
$$

由此可知，等比例降速具有以下效果：

1. 左右轮速度比例保持不变；
2. 左右轮旋转方向保持不变；
3. 机体线速度和偏航角速度同比例下降；
4. 底盘转弯半径和运动轨迹曲率保持不变；
5. 只降低底盘沿原轨迹运动的速度。

这里的“保持机体运动姿态”更准确地表示：保持底盘的瞬时运动方向和轨迹曲率不变，而不是保持底盘的绝对位置和偏航角不变。

### 4.6 电机最大转速宏定义

```c
/*
 * 电机转子的最大允许转速，单位为 rpm。
 * 该数值必须根据电机手册和驱动器配置进行设置。
 */
#define CHASSIS_DIFF_MOTOR_MAX_RPM (3000.0f)
```

### 4.7 等比例转速限制函数

```c
#include <math.h>

/**
 * @brief 对左右电机转速进行等比例限制。
 *
 * @param max_rpm   电机转子最大允许转速，单位为 rpm。
 * @param rpm_left  左电机转速输入及限制结果，单位为 rpm。
 * @param rpm_right 右电机转速输入及限制结果，单位为 rpm。
 *
 * @return 实际采用的缩放系数，范围为 (0, 1]。
 *
 * @note max_rpm 必须大于 0。
 * @note rpm_left 和 rpm_right 必须为有效指针。
 */
float chassis_diff_limit_motor_rpm(float max_rpm,
                                   float *rpm_left,
                                   float *rpm_right)
{
    float current_max_rpm;
    float k;
    float scale;

    /*
     * 电机转速包含方向，因此必须比较左右转速的绝对值。
     */
    current_max_rpm =
        fmaxf(fabsf(*rpm_left), fabsf(*rpm_right));

    /*
     * 当前左右电机均未超速，不需要进行缩放。
     * 同时避免静止状态下出现 0/0。
     */
    if (current_max_rpm <= max_rpm) {
        return 1.0f;
    }

    /*
     * k > 1 表示当前最大转速超过允许值。
     */
    k = current_max_rpm / max_rpm;

    /*
     * scale = 1/k，将最大转速降至允许值。
     */
    scale = 1.0f / k;

    /*
     * 左右电机必须使用同一个比例系数。
     */
    *rpm_left *= scale;
    *rpm_right *= scale;

    return scale;
}
```

### 4.8 在底盘运动学计算中调用

首先通过逆运动学计算左右电机目标转速：

```c
float left_motor_target_rpm;
float right_motor_target_rpm;
float motor_speed_scale;

chassis_diff_cmd(
    v_robot,
    w_robot,
    &left_motor_target_rpm,
    &right_motor_target_rpm
);
```

再执行等比例转速限制：

```c
motor_speed_scale =
    chassis_diff_limit_motor_rpm(
        CHASSIS_DIFF_MOTOR_MAX_RPM,
        &left_motor_target_rpm,
        &right_motor_target_rpm
    );
```

经过限制后：

```c
left_motor_target_rpm
right_motor_target_rpm
```

即为最终可以发送给左右电机驱动器的目标转速。

返回值 `motor_speed_scale` 可用于调试：

- `motor_speed_scale == 1.0f`：没有发生限速；
- `motor_speed_scale < 1.0f`：左右电机已经等比例降速。
## 5.附录
### 5.1 参考文献
[1] 两轮差速驱动运动模型 - 熊思录的文章 - 知乎 https://zhuanlan.zhihu.com/p/635908682  
[2] 机器人底盘两轮差速模型详解 https://www.bilibili.com/video/BV1Wk4y1w7rg/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb  
[3] 底盘运动学解析系统性教程（一）| 两轮差速小车 https://www.bilibili.com/video/BV1K1w9efEXm/?share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb