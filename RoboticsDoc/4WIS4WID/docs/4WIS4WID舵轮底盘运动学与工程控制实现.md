# 4WIS4WID舵轮底盘运动学与工程控制实现

# 1. 机器人坐标系、模块编号与符号约定

## 1.1 机体右手坐标系

![](<../assets/Fig.1 4WIS4WID舵轮底盘机体数学⽰意图与模块分布.png>)

*Fig.1 4WIS4WID舵轮底盘机体数学示意图与模块分布*

*Tab.1 机体固连右手坐标系*

| 符号 | 正方向或含义 |
| --- | --- |
| $+x_b$ | 机体前方 |
| $+y_b$ | 机体左侧 |
| $+z_b$ | 垂直底盘向上 |
| $W_z>0$ | 从 $+z_b$ 轴一侧朝坐标原点观察时，机体逆时针旋转 |

## 1.2 四个舵轮模块编号

*Tab.2 舵轮模块编号*

| 编号 | 模块 | 枚举 | CAN ID |
| --- | --- | --- | --- |
| 1 | FL 左前 | `APP_STEER_MODULE_FL` | 1 |
| 2 | RL 左后 | `APP_STEER_MODULE_RL` | 2 |
| 3 | RR 右后 | `APP_STEER_MODULE_RR` | 3 |
| 4 | FR 右前 | `APP_STEER_MODULE_FR` | 4 |

## 1.3 机械结构符号

*Tab.3 机械结构符号*

| 符号 | 含义 | 单位 |
| --- | --- | --- |
| $L$ | 前后轮中心之间的完整轴距 | m |
| $H$ | 左右轮中心之间的完整轮距 | m |
| $a$ | 半轴距，$a=L/2$ | m |
| $b$ | 半轮距，$b=H/2$ | m |

例如，当工程参数为：

$$
a=0.1675\ \mathrm{m}
$$

$$
b=0.200425\ \mathrm{m}
$$

则有对应代码：

代码块

```c
#define APP_STEER_HALF_WHEEL_BASE_M    (0.1675f)
#define APP_STEER_HALF_TRACK_WIDTH_M   (0.200425f)
```

## 1.4 运动学与控制符号

*Tab.4 运动学与控制符号*

| 符号 | 含义 | 单位 |
| --- | --- | --- |
| $\{W\}$ | 世界坐标系 | — |
| $\{B\}$ | 机体固连坐标系 | — |
| $x_W,\ y_W,\ z_W$ | 世界坐标系的三个坐标轴 | — |
| $x_b,\ y_b,\ z_b$ | 机体坐标系的三个坐标轴 | — |
| $\psi$ | 机体相对世界坐标系的偏航角，逆时针为正 | rad |
| $V_x\equiv{}^B V_x$ | 机体中心平动速度沿 $x_b$ 方向的分量 | m/s |
| $V_y\equiv{}^B V_y$ | 机体中心平动速度沿 $y_b$ 方向的分量 | m/s |
| $W_z$ | 机体绕 $z_b$ 轴的偏航角速度，逆时针为正 | rad/s |
| ${}^W\mathbf{V}$ | 世界坐标系中表达的机体中心平动速度向量 | m/s |
| ${}^B\mathbf{V}$ | 机体坐标系中表达的机体中心平动速度向量 | m/s |
| ${}^W V_x,\ {}^W V_y$ | ${}^W\mathbf{V}$ 沿 $x_W$、$y_W$ 方向的分量 | m/s |
| $\mathbf{r}_i$ | 从机体中心指向第 $i$ 个模块的位置向量 | m |
| $x_i$ | $\mathbf{r}_i$ 沿 $x_b$ 方向的分量 | m |
| $y_i$ | $\mathbf{r}_i$ 沿 $y_b$ 方向的分量 | m |
| $R_i=\lVert\mathbf{r}_i\rVert$ | 第 $i$ 个模块到机体中心的距离 | m |
| $\alpha_i$ | $\mathbf{r}_i$ 相对 $x_b$ 轴的夹角 | rad |
| $\mathbf{v}_{\mathrm{translation}}$ | 机体平动在各模块处产生的速度向量，在机体坐标系中表达 | m/s |
| $\mathbf{v}_{\mathrm{rotation},i}$ | 机体自转在第 $i$ 个模块处产生的切向速度向量，在机体坐标系中表达 | m/s |
| $\mathbf{v}_i$ | 第 $i$ 个模块的合速度向量，在机体坐标系中表达 | m/s |
| $v_{i,x}\equiv{}^B v_{i,x}$ | $\mathbf{v}_i$ 沿 $x_b$ 方向的分量 | m/s |
| $v_{i,y}\equiv{}^B v_{i,y}$ | $\mathbf{v}_i$ 沿 $y_b$ 方向的分量 | m/s |
| $V_i=\lVert\mathbf{v}_i\rVert$ | 第 $i$ 个模块的合速度大小 | m/s |
| $V_{i,\mathrm{cmd}}$ | 舵角折叠及轮毂反转后的带方向轮毂期望线速度 | m/s |
| $\theta_{i,\mathrm{raw}}$ | 由 $\operatorname{atan2}(v_{i,y},v_{i,x})$ 得到的第 $i$ 个模块在机体坐标系中的原始舵向角，范围为 $[-\pi,\pi]$ | rad |
| $\theta_{i,\mathrm{raw,motor}}$ | $\theta_{i,\mathrm{raw}}$ 叠加舵向零点补偿并完成角度环绕后，在电机坐标系中的原始目标角，尚未进行舵角折叠，范围为 $[-\pi,\pi]$ | rad |
| $\theta_{i,\mathrm{motor}}$ | 在电机坐标系中完成固定半平面折叠后，最终发送给第 $i$ 个舵向电机位置环的目标角，范围为 $[-\pi/2,\pi/2]$ | rad |
| $\theta_{i,\mathrm{motor,prev}}$ | 第 $i$ 个舵向电机上一时刻有效的最终目标角 | rad |
| $\delta_i$ | 第 $i$ 个舵向模块由机体舵向角转换到电机位置角时使用的零点补偿量，满足 $\theta_{i,\mathrm{raw,motor}}=\operatorname{wrap}_{[-\pi,\pi]}(\theta_{i,\mathrm{raw}}+\delta_i)$ | rad |
| $\varepsilon_v$ | 判断模块合速度是否可视为零的速度阈值 | m/s |
| $\sigma_{i,\mathrm{reverse}}$ | 根据 $\theta_{i,\mathrm{raw,motor}}$ 的折叠结果确定的轮毂反转符号，满足 $V_{i,\mathrm{cmd}}=\sigma_{i,\mathrm{reverse}}V_i$ | — |
| $r_{\mathrm{wheel}}$ | 轮毂半径 | m |
| $g_{\mathrm{drive}}$ | 驱动转速比，定义为电机转速与轮毂转速之比 | — |
| $\sigma_{i,\mathrm{mount}}$ | 第 $i$ 个驱动电机的机械安装极性补偿系数 | — |
| $n_i$ | 第 $i$ 个驱动电机统一限速前的目标转速 | rpm |
| $n_{\max}$ | 驱动电机允许的最大转速幅值 | rpm |
| $n_{\mathrm{peak}}$ | 四个驱动电机统一限速前目标转速的最大绝对值 | rpm |
| $s$ | 四轮统一等比例限速系数 | — |
| $n'_i$ | 第 $i$ 个驱动电机统一限速后的最终目标转速 | rpm |

视角以机体坐标系{B}为准时，其四个模块安装坐标为：

$$
\begin{aligned}
\mathrm{FL}:(x_1,y_1)&=(+a,+b)=\left(+\frac{L}{2},+\frac{H}{2}\right)\\
\mathrm{RL}:(x_2,y_2)&=(-a,+b)=\left(-\frac{L}{2},+\frac{H}{2}\right)\\
\mathrm{RR}:(x_3,y_3)&=(-a,-b)=\left(-\frac{L}{2},-\frac{H}{2}\right)\\
\mathrm{FR}:(x_4,y_4)&=(+a,-b)=\left(+\frac{L}{2},-\frac{H}{2}\right)
\end{aligned}\tag{1}
$$

# 2. 模块线速度合的组成

第 $i$ 个模块的线速度合由两部分组成：

$$
\boxed{\mathbf{v}_i=\mathbf{v}_{\mathrm{translation}}+\mathbf{v}_{\mathrm{rotation},i}}\tag{2}
$$

其中：

- $\mathbf{v}_{\mathrm{translation}}$：机体整体平动产生的速度；

- $\mathbf{v}_{\mathrm{rotation},i}$：机体自转在第 $i$ 个模块位置产生的切向线速度；

## 2.1 平动速度

机体平动速度为：

$$
\boxed{\mathbf{v}_{\mathrm{translation}}=\begin{bmatrix}V_x\\V_y\end{bmatrix}}\tag{3}
$$

注意注意注意！！！刚体纯平动时，车体上所有点的速度相同，所以四个模块都获得相同的平动速度：

$$
\mathbf{v}_{\mathrm{translation}}=\begin{bmatrix}V_x\\V_y\end{bmatrix}
$$

## 2.2 旋转速度的两个核心约束

机体绕中心自转时，第 $i$ 个模块到中心的距离保持不变：

$$
R_i=\sqrt{x_i^2+y_i^2}
$$

模块旋转线速度满足：

$$
\boxed{\lVert\mathbf{v}_{\mathrm{rotation},i}\rVert=|W_z|R_i}\tag{4}
$$

同时，旋转线速度必须与模块位置向量正交：

$$
\boxed{\mathbf{v}_{\mathrm{rotation},i}\perp\mathbf{r}_i}\tag{5}
$$

因此：

- 式（4）确定速度大小；

- 式（5）确定速度沿圆周切线；

- $W_z$ 的正负号确定顺时针或逆时针方向；

## 2.3 旋转速度分量推导

模块位置向量写成极坐标形式：

$$
\mathbf{r}_i=R_i\begin{bmatrix}\cos\alpha_i\\\sin\alpha_i\end{bmatrix}=\begin{bmatrix}x_i\\y_i\end{bmatrix}\tag{6}
$$

所以：

$$
x_i=R_i\cos\alpha_i\qquad y_i=R_i\sin\alpha_i
$$

当 $W_z>0$ 时，模块沿逆时针切向运动。径向单位向量为：

$$
\hat{\mathbf{r}}_i=\frac{\mathbf{r}_i}{R_i}=\begin{bmatrix}\cos\alpha_i\\\sin\alpha_i\end{bmatrix}
$$

二维向量逆时针旋转 $90^\circ$ 的旋转矩阵为：

$$
\mathbf{R}\left(\frac{\pi}{2}\right)=\begin{bmatrix}0&-1\\1&0\end{bmatrix}
$$

由于本文使用列向量，旋转矩阵从左侧乘以径向单位向量。因此，切向单位向量为：

$$
\begin{aligned}
\hat{\mathbf{t}}_i&=\mathbf{R}\left(\frac{\pi}{2}\right)\hat{\mathbf{r}}_i\\
&=\begin{bmatrix}0&-1\\1&0\end{bmatrix}\begin{bmatrix}\cos\alpha_i\\\sin\alpha_i\end{bmatrix}\\
&=\begin{bmatrix}-\sin\alpha_i\\\cos\alpha_i\end{bmatrix}
\end{aligned}
$$

因此旋转线速度向量为：

$$
\begin{aligned}
\mathbf{v}_{\mathrm{rotation},i}&=W_zR_i\hat{\mathbf{t}}_i\\
&=W_zR_i\begin{bmatrix}-\sin\alpha_i\\\cos\alpha_i\end{bmatrix}\\
&=W_z\begin{bmatrix}-R_i\sin\alpha_i\\R_i\cos\alpha_i\end{bmatrix}\\
&=W_z\begin{bmatrix}-y_i\\x_i\end{bmatrix}
\end{aligned}\tag{7}
$$

最终：

$$
\boxed{\mathbf{v}_{\mathrm{rotation},i}=\begin{bmatrix}-W_zy_i\\W_zx_i\end{bmatrix}}\tag{8}
$$

这里 $R_i$ 没有消失，而是通过：

$$
R_i\sin\alpha_i=y_i,\qquad R_i\cos\alpha_i=x_i
$$

变量代换隐藏在模块安装坐标 $x_i$、$y_i$ 中。

## 2.4 平动速度与旋转速度叠加

将式（3）和式（8）相加：

$$
\begin{aligned}
\mathbf{v}_i&=\begin{bmatrix}V_x\\V_y\end{bmatrix}+\begin{bmatrix}-W_zy_i\\W_zx_i\end{bmatrix}\\
&=\begin{bmatrix}V_x-W_zy_i\\V_y+W_zx_i\end{bmatrix}
\end{aligned}\tag{9}
$$

因此，第 $i$ 个模块的通用逆运动学公式为：

$$
\boxed{v_{i,x}=V_x-W_zy_i}\tag{10}
$$

$$
\boxed{v_{i,y}=V_y+W_zx_i}\tag{11}
$$

- 其中，$V_x$、$V_y$、$W_z$ 均为上位机给定的期望运动量，属于已知量；

- $x_i$、$y_i$ 为第 $i$ 个模块相对机体中心的安装坐标，可通过机械尺寸测量和标定获得，同样属于已知量；

- 给定底盘期望运动后，可由式（10）和式（11）直接求得各模块的速度分量 $v_{i,x}$、$v_{i,y}$；

# 3. 四个轮组的分模块数学表达式推导

下面将四个模块坐标分别代入式（10）和式（11）

## 3.1 模块 1：FL 左前

FL 坐标为：

$$
x_1=+a,\qquad y_1=+b
$$

代入通用公式：

$$
v_{1,x}=V_x-W_z(+b)
$$

$$
v_{1,y}=V_y+W_z(+a)
$$

得到：

$$
\boxed{\begin{aligned}v_{\mathrm{FL},x}&=V_x-W_zb\\v_{\mathrm{FL},y}&=V_y+W_za\end{aligned}}\tag{12}
$$

当 $W_z>0$ 时，FL 的旋转速度方向为“向后、向左”。

对应代码：

代码块

```c
fl_vx = vx - wz * APP_STEER_HALF_TRACK_WIDTH_M;
fl_vy = vy + wz * APP_STEER_HALF_WHEEL_BASE_M;
```

## 3.2 模块 2：RL 左后

RL 坐标为：

$$
x_2=-a,\qquad y_2=+b
$$

代入：

$$
v_{2,x}=V_x-W_z(+b)
$$

$$
v_{2,y}=V_y+W_z(-a)
$$

得到：

$$
\boxed{\begin{aligned}v_{\mathrm{RL},x}&=V_x-W_zb\\v_{\mathrm{RL},y}&=V_y-W_za\end{aligned}}\tag{13}
$$

当 $W_z>0$ 时，RL 的旋转速度方向为“向后、向右”。

对应代码：

代码块

```c
rl_vx = vx - wz * APP_STEER_HALF_TRACK_WIDTH_M;
rl_vy = vy - wz * APP_STEER_HALF_WHEEL_BASE_M;
```

## 3.3 模块 3：RR 右后

RR 坐标为：

$$
x_3=-a,\qquad y_3=-b
$$

代入：

$$
v_{3,x}=V_x-W_z(-b)
$$

$$
v_{3,y}=V_y+W_z(-a)
$$

得到：

$$
\boxed{\begin{aligned}v_{\mathrm{RR},x}&=V_x+W_zb\\v_{\mathrm{RR},y}&=V_y-W_za\end{aligned}}\tag{14}
$$

当 $W_z>0$ 时，RR 的旋转速度方向为“向前、向右”。

对应代码：

代码块

```c
rr_vx = vx + wz * APP_STEER_HALF_TRACK_WIDTH_M;
rr_vy = vy - wz * APP_STEER_HALF_WHEEL_BASE_M;
```

## 3.4 模块 4：FR 右前

FR 坐标为：

$$
x_4=+a,\qquad y_4=-b
$$

代入：

$$
v_{4,x}=V_x-W_z(-b)
$$

$$
v_{4,y}=V_y+W_z(+a)
$$

得到：

$$
\boxed{\begin{aligned}v_{\mathrm{FR},x}&=V_x+W_zb\\v_{\mathrm{FR},y}&=V_y+W_za\end{aligned}}\tag{15}
$$

当 $W_z>0$ 时，FR 的旋转速度方向为“向前、向左”。

对应代码：

代码块

```c
fr_vx = vx + wz * APP_STEER_HALF_TRACK_WIDTH_M;
fr_vy = vy + wz * APP_STEER_HALF_WHEEL_BASE_M;
```

## 3.5 四个轮组的速度分量表达式汇总

四个轮组的数学表达式汇总：

$$
\boxed{\begin{aligned}
v_{\mathrm{FL},x}&=V_x-W_zb & v_{\mathrm{FL},y}&=V_y+W_za\\
v_{\mathrm{RL},x}&=V_x-W_zb & v_{\mathrm{RL},y}&=V_y-W_za\\
v_{\mathrm{RR},x}&=V_x+W_zb & v_{\mathrm{RR},y}&=V_y-W_za\\
v_{\mathrm{FR},x}&=V_x+W_zb & v_{\mathrm{FR},y}&=V_y+W_za
\end{aligned}}\tag{16}
$$

# 4. 由速度分量方程组成整体矩阵

先将八个标量方程写成标准形式：

$$
\begin{aligned}
v_{\mathrm{FL},x}&=1*V_x+0*V_y-b*W_z & v_{\mathrm{FL},y}&=0*V_x+1*V_y+a*W_z\\
v_{\mathrm{RL},x}&=1*V_x+0*V_y-b*W_z & v_{\mathrm{RL},y}&=0*V_x+1*V_y-a*W_z\\
v_{\mathrm{RR},x}&=1*V_x+0*V_y+b*W_z & v_{\mathrm{RR},y}&=0*V_x+1*V_y-a*W_z\\
v_{\mathrm{FR},x}&=1*V_x+0*V_y+b*W_z & v_{\mathrm{FR},y}&=0*V_x+1*V_y+a*W_z
\end{aligned}
$$

将每个方程中 $V_x$、$V_y$、$W_z$ 的系数逐行排列，得到：

$$
\boxed{\begin{bmatrix}
v_{\mathrm{FL},x}\\v_{\mathrm{FL},y}\\v_{\mathrm{RL},x}\\v_{\mathrm{RL},y}\\v_{\mathrm{RR},x}\\v_{\mathrm{RR},y}\\v_{\mathrm{FR},x}\\v_{\mathrm{FR},y}
\end{bmatrix}=\begin{bmatrix}
1&0&-b\\0&1&+a\\1&0&-b\\0&1&-a\\1&0&+b\\0&1&-a\\1&0&+b\\0&1&+a
\end{bmatrix}\begin{bmatrix}V_x\\V_y\\W_z\end{bmatrix}}\tag{17}
$$

该矩阵由四个模块对应的八个速度分量方程逐行组成，式（17）可以进一步写成简洁的线性映射形式：

$$
\mathbf{v}_{\mathrm{modules}}=\mathbf{A}_{\mathrm{chassis}}\mathbf{u}_{\mathrm{command}}
$$

其中：

$$
\mathbf{v}_{\mathrm{modules}}=[v_{\mathrm{FL},x}\quad v_{\mathrm{FL},y}\quad v_{\mathrm{RL},x}\quad v_{\mathrm{RL},y}\quad v_{\mathrm{RR},x}\quad v_{\mathrm{RR},y}\quad v_{\mathrm{FR},x}\quad v_{\mathrm{FR},y}]^{\mathrm{T}}
$$

为等式左侧的八个待求速度分量；

其中：

$$
\mathbf{u}_{\mathrm{command}}=[V_x\quad V_y\quad W_z]^{\mathrm{T}}
$$

为上位机——例如遥控，给定的三个已知期望运动量；

其中：

$$
\mathbf{A}_{\mathrm{chassis}}
$$

为机械结构参数 $a$、$b$ 及各模块安装位置的正负号确定的 $8\times3$ 系数矩阵。即底盘机械结构确定后，$a$、$b$ 均为已知常量，因此该系数矩阵也为已知量；

式（17）不是通过矩阵求逆来解方程，而是直接将已知的系数矩阵与已知的期望运动量相乘，得到左侧八个待求速度分量，即常言的“利用已知求未知”；

这一思路可拓展到 C 语言函数的编写，例如，函数传入 $V_x$、$V_y$、$W_z$ 三个参数，机械结构参数 $a$、$b$ 以及由它们组成的固定系数矩阵可通过宏定义或 `static const` 只读数组固化，再按照系数矩阵的各行依次计算八个模块速度分量；

知道各个模块的速度分量后，将每个模块对应的两个速度分量作为一组，计算该模块的合线速度大小与原始舵向角：

$$
V_i=\sqrt{v_{i,x}^2+v_{i,y}^2}
$$

$$
\theta_{i,\mathrm{raw}}=\operatorname{atan2}(v_{i,y},v_{i,x})
$$

# 5. 模块线速度合大小与舵向角

由上文可得到：

$$
v_{i,x},\qquad v_{i,y}
$$

模块线速度合模大小为：

$$
\boxed{V_i=\sqrt{v_{i,x}^2+v_{i,y}^2}}\tag{18}
$$

轮毂原始舵向角为：

$$
\boxed{\theta_{i,\mathrm{raw}}=\operatorname{atan2}(v_{i,y},v_{i,x})}\tag{19}
$$

轮毂滚动方向必须与模块线速度合共线，否则会产生侧向滑动，即轮毂朝向方向应该为该轮毂的线速度合矢量方向。

当模块线速度合接近零时，不再计算新的原始舵向角，而是令轮毂期望线速度为零，并保持上一时刻有效的电机位置:

$$
V_i<\varepsilon_v\Longrightarrow V_{i,\mathrm{cmd}}=0,\qquad \theta_{i,\mathrm{motor}}=\theta_{i,\mathrm{motor,prev}}
$$

对应的工程处理逻辑为：

代码块

```c
speed_i = sqrtf(vx_i * vx_i + vy_i * vy_i);

if (speed_i < speed_epsilon_mps)
{
    speed_cmd_i = 0.0f;
    steer_angle_cmd_i = steer_angle_cmd_i_last;
}
else
{
    raw_angle_i = atan2f(vy_i, vx_i);
    //......
    steer_angle_cmd_i_last = steer_angle_cmd_i;
}
```

# 6. 舵向零点补偿、舵角折叠与轮毂反转

由模块速度分量可得到第 $i$ 个模块在机体坐标系中的原始舵向角：

$$
\theta_{i,\mathrm{raw}}=\operatorname{atan2}(v_{i,y},v_{i,x})
$$

其中：

$$
\theta_{i,\mathrm{raw}}\in[-\pi,\pi]
$$

$\theta_{i,\mathrm{raw}}$ 表示轮毂期望滚动方向在机体坐标系中的角度，其零角方向为机体坐标系的 $+x_b$ 方向。

## 6.1 原始舵向角转换到电机坐标系

> 注意：由于舵向电机的机械安装零点不一定与机体坐标系的 $+x_b$ 方向重合，应为每个舵向模块标定零点补偿量 $\delta_i$，实现电控代码软件补偿或在官方上位机手动重新标定零点位置（去上位机点一下标定零点即可）

将机体坐标系中的原始舵向角转换为电机坐标系中的原始目标角：

$$
\boxed{\theta_{i,\mathrm{raw,motor}}=\operatorname{wrap}_{[-\pi,\pi]}(\theta_{i,\mathrm{raw}}+\delta_i)}
$$

其中：

- $\theta_{i,\mathrm{raw}}$ 为机体坐标系中的原始舵向角；

- $\delta_i$ 为第 $i$ 个舵向模块的零点补偿量；

- $\theta_{i,\mathrm{raw,motor}}$ 为零点补偿后电机坐标系中的原始目标角；

## 6.2 在电机坐标系中执行固定半平面折叠

本文采用固定半平面舵角折叠策略，将电机坐标系中的原始目标角约束到：

$$
\left[-\frac{\pi}{2},+\frac{\pi}{2}\right]
$$

最终发送给舵向电机位置环的目标角为：

$$
\boxed{\theta_{i,\mathrm{motor}}=\begin{cases}
\theta_{i,\mathrm{raw,motor}}-\pi,&\theta_{i,\mathrm{raw,motor}}>\dfrac{\pi}{2}\\
\theta_{i,\mathrm{raw,motor}}+\pi,&\theta_{i,\mathrm{raw,motor}}<-\dfrac{\pi}{2}\\
\theta_{i,\mathrm{raw,motor}},&|\theta_{i,\mathrm{raw,motor}}|\leq\dfrac{\pi}{2}
\end{cases}}
$$

因此：

$$
\theta_{i,\mathrm{motor}}\in\left[-\frac{\pi}{2},+\frac{\pi}{2}\right]
$$

这里的折叠判断必须基于零点补偿后的电机目标角：

$$
\theta_{i,\mathrm{raw,motor}}
$$

而不能直接基于机体坐标系中的：

$$
\theta_{i,\mathrm{raw}}
$$

因为实际位置环接收和执行的是电机坐标系中的角度命令，舵向角是否超过允许折叠区间，应当依据电机坐标系中的目标角判断。

## 6.3 轮毂反转符号

当电机目标角增加或减小 $\pi$ 后，轮毂滚动方向单位向量会反向。为了保持轮组最终产生的速度向量不变，必须同步反转轮毂期望速度。

定义轮毂反转符号：

$$
\boxed{\sigma_{i,\mathrm{reverse}}=\begin{cases}
-1,&|\theta_{i,\mathrm{raw,motor}}|>\dfrac{\pi}{2}\\
+1,&|\theta_{i,\mathrm{raw,motor}}|\leq\dfrac{\pi}{2}
\end{cases}}
$$

则带方向的轮毂期望线速度为：

$$
\boxed{V_{i,\mathrm{cmd}}=\sigma_{i,\mathrm{reverse}}V_i}
$$

其中：

- $V_i\geq0$ 为第 $i$ 个模块的合速度大小；

- $V_{i,\mathrm{cmd}}$ 的正负号表示轮毂期望转动方向；

- $\theta_{i,\mathrm{motor}}$ 为最终发送给舵向电机位置环的目标角度。

## 6.4 速度向量等效性（小证明）

设轮毂实际滚动方向对应的机体舵向角为 $\theta$，则舵向角改变 $\pi$ 并同步反转轮毂速度后，有：

$$
V_i\begin{bmatrix}\cos\theta\\\sin\theta\end{bmatrix}=(-V_i)\begin{bmatrix}\cos(\theta+\pi)\\\sin(\theta+\pi)\end{bmatrix}
$$

因此，舵向角折叠与轮毂速度反转不会改变轮组最终产生的速度向量。

## 6.5 固定半平面折叠策略的限制

本文的候选解选择仅由零点补偿后的电机目标角所在区间决定，即仅判断：

$$
|\theta_{i,\mathrm{raw,motor}}|>\frac{\pi}{2}
$$

该策略未使用舵向电机当前反馈角度，因此不等同于基于当前舵向角的动态最短路径优化，也不能保证每次舵向电机的转动距离最小。

> 舵向电机在此方案下依然有概率大角度转动180度，应该注意上位机给定量的选取和优化！！！

## 6.6 工程处理逻辑

对应的工程实现流程为：

> 计算模块原始舵向角  
> theta_i_raw = atan2(v_i_y, v_i_x)  
> ↓  
> 叠加舵向零点补偿并执行角度环绕  
> theta_i_raw_motor =  
> &nbsp;&nbsp;&nbsp;&nbsp;wrap(theta_i_raw + delta_i)  
> ↓  
> 在电机坐标系中执行固定半平面折叠  
> If theta_i_raw_motor > +pi/2:  
> &nbsp;&nbsp;&nbsp;&nbsp;theta_i_motor = theta_i_raw_motor - pi  
> &nbsp;&nbsp;&nbsp;&nbsp;hub_reverse_flag = 1  
>  
> Else if theta_i_raw_motor < -pi/2:  
> &nbsp;&nbsp;&nbsp;&nbsp;theta_i_motor = theta_i_raw_motor + pi  
> &nbsp;&nbsp;&nbsp;&nbsp;hub_reverse_flag = 1  
>  
> Else :  
> &nbsp;&nbsp;&nbsp;&nbsp;theta_i_motor = theta_i_raw_motor  
> &nbsp;&nbsp;&nbsp;&nbsp;hub_reverse_flag = 0  
> ↓  
> 根据 hub_reverse_flag 确定轮毂期望速度方向

# 7. 轮毂线速度转换为 RPM

*Tab.5 实际轮毂参数*

| 参数 | 数值 |
| --- | --- |
| 轮毂半径 | 0.075 m |
| 驱动减速比 | 1 : 1 |
| 最大允许转速幅值 | $n_{\max}=200\ \mathrm{rpm}$ |
| 允许目标转速范围 | $[-n_{\max},+n_{\max}]=[-200,+200]\ \mathrm{rpm}$ |

将视角转变为轮毂（本来视角是从天上朝下看底盘，现在侧过来看底盘），轮毂线速度合可转换为轮毂电机转速：

$$
\boxed{n_i=V_{i,\mathrm{cmd}}*\frac{60g_{\mathrm{drive}}}{2\pi r_{\mathrm{wheel}}}*\sigma_{i,\mathrm{mount}}}\tag{20}
$$

# 8. 四轮统一等比例限速

定义四个驱动电机统一限速前目标转速的最大绝对值：

$$
n_{\mathrm{peak}}=\max(|n_{\mathrm{FL}}|,|n_{\mathrm{RL}}|,|n_{\mathrm{RR}}|,|n_{\mathrm{FR}}|)
$$

统一缩放系数：

$$
s=\begin{cases}
1,&n_{\mathrm{peak}}\leq n_{\max}\\
\dfrac{n_{\max}}{n_{\mathrm{peak}}},&n_{\mathrm{peak}}>n_{\max}
\end{cases}\tag{21}
$$

这里的 $n_{\mathrm{FL}}$、$n_{\mathrm{RL}}$、$n_{\mathrm{RR}}$、$n_{\mathrm{FR}}$ 均为限速前的目标转速，并非电机当前实测转速；

四轮最终转速：

$$
\boxed{n'_i=sn_i}\tag{22}
$$

四轮使用同一个 $s$，可以保持：

- 四轮速度比例不变；

- 模块速度方向不变；

- 舵向角不变；

- 机体瞬时运动比例不变；

- 瞬时旋转中心和瞬时曲率不变；

# 9. 工程代码对应

当前源码中的四轮逆运动学：

代码块

```c
/* FL: x=+base/2, y=+track/2 */
fl_vx = vx - wz * APP_STEER_HALF_TRACK_WIDTH_M;
fl_vy = vy + wz * APP_STEER_HALF_WHEEL_BASE_M;

/* RL: x=-base/2, y=+track/2 */
rl_vx = vx - wz * APP_STEER_HALF_TRACK_WIDTH_M;
rl_vy = vy - wz * APP_STEER_HALF_WHEEL_BASE_M;

/* RR: x=-base/2, y=-track/2 */
rr_vx = vx + wz * APP_STEER_HALF_TRACK_WIDTH_M;
rr_vy = vy - wz * APP_STEER_HALF_WHEEL_BASE_M;

/* FR: x=+base/2, y=-track/2 */
fr_vx = vx + wz * APP_STEER_HALF_TRACK_WIDTH_M;
fr_vy = vy + wz * APP_STEER_HALF_WHEEL_BASE_M;
```

# 10. 控制思路总结

舵轮逆运动学控制流程如下：

> 标定右手坐标系和四个模块坐标  
> ↓  
> 机体平动速度  
> [Vx, Vy]  
> &nbsp;&nbsp;&nbsp;&nbsp;+  
> 机体自转产生的模块切向速度  
> [-Wz*yi, Wz*xi]  
> ↓  
> 第 i 个模块合速度  
> [Vx - Wz*yi, Vy + Wz*xi]  
> ↓  
> 计算速度大小与舵向角  
> Vi = sqrt(vx,i² + vy,i²)  
> theta_i,raw = atan2(vy,i, vx,i)  
> ↓  
> 零点补偿，转换为电机坐标中的原始目标角  
> ↓  
> theta_i,raw,motor = wrap(theta_i,raw + delta_i)  
> ↓  
> 按机械范围折叠电机目标角，并同步反转轮毂速度  
> ↓  
> [theta_i,motor, V_i,cmd]  
> ├── theta_i,motor → FDCAN 舵向电机位置环  
> │  
> └── V_i,cmd → 线速度转换为 RPM → n_i  
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↓  
> &nbsp;&nbsp;&nbsp;&nbsp;四轮统一等比例限速  
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↓  
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;n_i'  
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↓  
> &nbsp;&nbsp;&nbsp;&nbsp;RS485 轮毂电机速度环

核心公式为：

$$
\boxed{\begin{aligned}
v_{i,x}&=V_x-W_zy_i\\
v_{i,y}&=V_y+W_zx_i\\
V_i&=\sqrt{v_{i,x}^2+v_{i,y}^2}\\
\theta_{i,\mathrm{raw}}&=\operatorname{atan2}(v_{i,y},v_{i,x})
\end{aligned}}\tag{23}
$$

# 11. 平动小陀螺

## 11.1 数学推导

注意，前文中的四舵轮逆运动学默认输入 $V_x$、$V_y$、$W_z$ 均表达在机体坐标系中。

因此，若持续给定：

$$
{}^B V_x=\text{常数},\qquad {}^B V_y=\text{常数}
$$

同时：

$$
W_z\neq0
$$

则平动方向在世界坐标系中也会随之旋转。

具体数学证明如下：

标准二维逆时针旋转矩阵为：

$$
\mathbf{R}(\psi)=\begin{bmatrix}\cos\psi&-\sin\psi\\\sin\psi&\cos\psi\end{bmatrix}
$$

由于标准二维逆时针旋转矩阵为正交矩阵，有：

$$
\mathbf{R}^{-1}(\psi)=\mathbf{R}^{\mathrm{T}}(\psi)=\mathbf{R}(-\psi)=\begin{bmatrix}\cos\psi&\sin\psi\\-\sin\psi&\cos\psi\end{bmatrix}
$$

采用如下旋转矩阵符号约定：

$$
{}^A\mathbf{R}_B
$$

为将同一个物理向量在坐标系 $\{B\}$ 中的坐标表达，转换为该向量在坐标系 $\{A\}$ 中的坐标表达。其中，左上标表示目标坐标系，右下标表示原始坐标系。

在忽略轮胎滑移和执行器跟踪误差的理想条件下，底盘实际执行的机体系平动速度为：

$$
\boxed{{}^B\mathbf{V}_{\mathrm{actual}}={}^B\mathbf{V}_{\mathrm{cmd}}}\tag{24}
$$

当机体相对世界坐标系逆时针旋转偏航角 $\psi$ 时，机体在世界系视角下的实际平动速度为：

$$
\boxed{{}^W\mathbf{V}_{\mathrm{actual}}={}^W\mathbf{R}_B(\psi){}^B\mathbf{V}_{\mathrm{cmd}}}\tag{25}
$$

由于：

$$
\boxed{{}^W\mathbf{R}_B(\psi)=\mathbf{R}(\psi)}\tag{26}
$$

将（26）代入（25），有：

$$
\boxed{{}^W\mathbf{V}_{\mathrm{actual}}={}^W\mathbf{R}_B(\psi){}^B\mathbf{V}_{\mathrm{cmd}}=\mathbf{R}(\psi){}^B\mathbf{V}_{\mathrm{cmd}}}\tag{27}
$$

式（27）表明，机体系中的速度命令转换到世界坐标系后，会自然左乘旋转矩阵 $\mathbf{R}(\psi)$——若直接将世界系期望速度作为机体系速度命令，则实际世界速度将随机体偏航角发生旋转。

为使实际世界系速度始终等于给定的世界系期望速度，令：

$$
\boxed{{}^W\mathbf{V}_{\mathrm{actual}}={}^W\mathbf{V}_{\mathrm{des}}}\tag{28}
$$

将（27）代入（28），有：

$$
\boxed{{}^W\mathbf{V}_{\mathrm{actual}}={}^W\mathbf{V}_{\mathrm{des}}={}^W\mathbf{R}_B(\psi){}^B\mathbf{V}_{\mathrm{cmd}}=\mathbf{R}(\psi){}^B\mathbf{V}_{\mathrm{cmd}}}\tag{29}
$$

式（29）对 ${}^W\mathbf{R}_B(\psi)={}^W\mathbf{R}_B(\psi)$ 求逆，可推出传入机体的控制量应该为：

$$
\boxed{{}^B\mathbf{V}_{\mathrm{cmd}}={}^B\mathbf{R}_W(\psi){}^W\mathbf{V}_{\mathrm{des}}=\mathbf{R}(-\psi){}^W\mathbf{V}_{\mathrm{des}}}\tag{30}
$$

当世界系期望平动速度 ${}^W\mathbf{V}_{\mathrm{des}}$ 有左乘上述 $\mathbf{R}(-\psi)$ 约束，即联立式（28）、式（29）、式（30），可推出最终实际世界速度 ${}^W\mathbf{V}_{\mathrm{actual}}$ 为：

$$
\boxed{\begin{aligned}
{}^W\mathbf{V}_{\mathrm{actual}}&={}^W\mathbf{V}_{\mathrm{des}}\\
&={}^W\mathbf{R}_B(\psi){}^B\mathbf{V}_{\mathrm{cmd}}\\
&={}^W\mathbf{R}_B(\psi){}^B\mathbf{R}_W(\psi){}^W\mathbf{V}_{\mathrm{des}}
\end{aligned}}\tag{31}
$$

又因为：

$$
\boxed{{}^W\mathbf{R}_B(\psi)=\mathbf{R}(\psi),\ {}^B\mathbf{R}_W(\psi)=\mathbf{R}(-\psi)}\tag{32}
$$

将（32）代入(31)，完全展开有：

$$
\boxed{\begin{aligned}
{}^W\mathbf{V}_{\mathrm{actual}}&={}^W\mathbf{R}_B(\psi){}^B\mathbf{R}_W(\psi){}^W\mathbf{V}_{\mathrm{des}}\\
&=\mathbf{R}(\psi)\mathbf{R}(-\psi){}^W\mathbf{V}_{\mathrm{des}}
\end{aligned}}\tag{33}
$$

由于：

$$
\boxed{\mathbf{R}(\psi)\mathbf{R}(-\psi)=\mathbf{I}}\tag{34}
$$

将（34）代入（33），有：

$$
\boxed{{}^W\mathbf{V}_{\mathrm{actual}}={}^W\mathbf{V}_{\mathrm{des}}}\tag{35}
$$

因此，当有约束式（28）存在的时候，将控制量变为式（30），可推出式（35）结论——机体在自转小陀螺之时，其平动方向依旧为世界系期望方向。

"VCR"如下：

> 【粉红舵轮小陀螺】 https://www.bilibili.com/video/BV1LHN96xExW/?  
> share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb

由此观知，可使用 $\mathbf{R}(-\psi)$ 对世界系期望平动速度 ${}^W\mathbf{V}_{\mathrm{des}}$ 进行逆旋转补偿，从而抵消机体姿态旋转自然产生的 $\mathbf{R}(\psi)$，最终使底盘在自转过程中仍保持世界坐标系下的期望平动方向。

这种控制方式通常称为：

> 场地坐标控制  
> Field-oriented control  
> Field-centric control

## 11.2 抽象例子——沿世界坐标系正 $x_W$ 方向运动

设系统上电后建立世界坐标系，并持续给定沿世界坐标系 $+x_W$ 方向的期望速度：

$$
{}^W\mathbf{V}=\begin{bmatrix}V\\0\end{bmatrix},\qquad V>0
$$

同时，底盘以正偏航角速度开始逆时针旋转，使机体偏航角逐渐变为：

$$
\psi>0
$$

将世界系速度转换到机体系：

$$
\begin{aligned}
{}^B\mathbf{V}&={}^B\mathbf{R}_W(\psi){}^W\mathbf{V}\\
&=\begin{bmatrix}\cos\psi&\sin\psi\\-\sin\psi&\cos\psi\end{bmatrix}\begin{bmatrix}V\\0\end{bmatrix}\\
&=\begin{bmatrix}V\cos\psi\\-V\sin\psi\end{bmatrix}
\end{aligned}
$$

因此：

$$
\boxed{{}^B V_x=V\cos\psi}
$$

$$
\boxed{{}^B V_y=-V\sin\psi}
$$

## 11.3 典型例子

设世界坐标系中的目标平动速度始终为：

$$
{}^W\mathbf{V}=\begin{bmatrix}1\\0\end{bmatrix}\ \mathrm{m/s}
$$

表示底盘始终沿世界 $x_W$ 正方向运动。

**（1）机体偏航角为 $0^\circ$**

当：

$$
\psi=0^\circ
$$

有：

$$
{}^B\mathbf{V}=\begin{bmatrix}1&0\\0&1\end{bmatrix}\begin{bmatrix}1\\0\end{bmatrix}=\begin{bmatrix}1\\0\end{bmatrix}
$$

此时机体应沿自身前方平动。

**（2）机体逆时针旋转到 $90^\circ$**

当：

$$
\psi=90^\circ
$$

有：

$$
\begin{aligned}
{}^B\mathbf{V}&=\begin{bmatrix}0&1\\-1&0\end{bmatrix}\begin{bmatrix}1\\0\end{bmatrix}\\
&=\begin{bmatrix}0\\-1\end{bmatrix}
\end{aligned}
$$

此时机体自身前方已经指向世界左侧

要继续沿世界 $x_W$ 正方向运动，底盘必须沿自身右侧平移：

$$
{}^B V_x=0,\qquad {}^B V_y=-1
$$

这正是逆旋转矩阵计算得到的结果。

## 11.4 C 代码实现

代码块

```c
#include <math.h>
#include <stddef.h>
/**
 * @brief     将世界坐标系平动速度转换为机体坐标系平动速度。
 *
 坐标约定：
 世界系和机体系均为右手坐标系；
 yaw_rad > 0 表示机体相对世界坐标系逆时针旋转；
 vx_world_mps、vy_world_mps 为世界坐标系期望速度；
 vx_body_mps、vy_body_mps 为四舵轮逆运动学所需的机体系速度。
 */
static void app_chassis_world_to_body_velocity(
    float vx_world_mps,
    float vy_world_mps,
    float yaw_rad,
    float *vx_body_mps,
    float *vy_body_mps)
{
    float cos_yaw;
    float sin_yaw;

    if ((vx_body_mps == NULL)
        || (vy_body_mps == NULL)) {
        return;
    }

    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    /*
     * [Vx_body]     [ cos(yaw)   sin(yaw)] [Vx_world]
     * [Vy_body] = [-sin(yaw)     cos(yaw)] [Vy_world]
     *
     * 即：
     * V_body = R(-yaw) * V_world
     */
    *vx_body_mps =
        vx_world_mps * cos_yaw
        + vy_world_mps * sin_yaw;

    *vy_body_mps =
        -vx_world_mps * sin_yaw
        + vy_world_mps * cos_yaw;
}
```

与现有底盘命令函数衔接：

代码块

```c
void app_steer_chassis_field_cmd(
    float vx_world_mps,
    float vy_world_mps,
    float wz_robot_rad_s,
    float yaw_robot_rad)
{
    float vx_body_mps;
    float vy_body_mps;

    app_chassis_world_to_body_velocity(
        vx_world_mps,
        vy_world_mps,
        yaw_robot_rad,
        &vx_body_mps,
        &vy_body_mps
    );

    app_steer_chassis_cmd(
        vx_body_mps,
        vy_body_mps,
        wz_robot_rad_s
    );
}
```

其中：

- `vx_world_mps`、`vy_world_mps` 决定世界坐标系中的平动方向；

- `wz_robot_rad_s` 决定机体自身旋转速度；

- `yaw_robot_rad` 由 IMU、姿态解算器或定位系统提供；

- `app_steer_chassis_cmd()` 继续使用原有机体系四舵轮逆运动学

## 11.5 航向角零点标定

场地坐标控制依赖偏航角 $\psi$，因此必须明确零点。

设 IMU 当前偏航角为：

$$
\psi_{\mathrm{imu}}
$$

系统启动或人工标定时记录：

$$
\psi_{\mathrm{zero}}
$$

用于控制的相对偏航角为：

$$
\boxed{\psi=\operatorname{wrap}_{[-\pi,\pi]}(\psi_{\mathrm{imu}}-\psi_{\mathrm{zero}})}
$$

## 11.6 需要重点检查的符号问题

**（1）偏航角正方向**

本文规定：

$$
\psi>0
$$

和：

$$
W_z>0
$$

均表示逆时针。

若 IMU 输出顺时针为正，则应先取反。

**（2）世界系到机体系必须使用 $\mathbf{R}(-\psi)$**

正确形式：

$$
{}^B\mathbf{V}=\mathbf{R}(-\psi){}^W\mathbf{V}
$$

若错误使用：

$$
\mathbf{R}(+\psi)
$$

底盘平动方向会随偏航角向错误方向补偿，通常表现为机体越旋转，世界方向偏差越大。

**（3）只旋转平动速度，不旋转 $W_z$**

旋转矩阵只处理：

$$
V_x,\qquad V_y
$$

因为它们是二维线速度向量。

偏航角速度：

$$
W_z
$$

是绕平面法向轴的标量，在二维坐标旋转下保持不变，不需要与 $V_x$、$V_y$ 一起乘二维旋转矩阵。

## 11.7 机体坐标控制与场地坐标控制对比

*Tab.6 机体坐标控制与场地坐标控制对比*

| 控制方式 | 输入速度保持不变时的含义 | 机体自转时世界平动方向 |
| --- | --- | --- |
| 机体坐标控制 | 始终沿机体自身固定方向平动 | 随机体朝向旋转 |
| 场地坐标控制 | 始终沿世界固定方向平动 | 保持不变 |

机体坐标控制直接使用：

$$
\begin{bmatrix}{}^B V_x\\{}^B V_y\end{bmatrix}
$$

场地坐标控制则还应执行：

$$
\begin{bmatrix}{}^B V_x\\{}^B V_y\end{bmatrix}=\mathbf{R}(-\psi)\begin{bmatrix}{}^W V_x\\{}^W V_y\end{bmatrix}
$$

场地坐标控制与四舵轮逆运动学的衔接流程为：

> 世界坐标系平动命令 ${}^W\mathbf{V}$  
> ↓  
> 读取当前偏航角并进行零点补偿，得到相对偏航角 $\psi$  
> ↓  
> 世界坐标系到机体坐标系的速度变换  
> ${}^B\mathbf{R}_W(\psi)=\mathbf{R}(-\psi)$  
> ↓  
> 机体坐标系平动命令  
> ${}^B\mathbf{V}={}^B\mathbf{R}_W(\psi){}^W\mathbf{V}$  
> ↓  
> 与机体偏航角速度 $W_z$ 一同进入四舵轮逆运动学  
> ↓  
> 计算各轮组的合速度 $V_i$ 与原始舵向角 $\theta_{i,\mathrm{raw}}$  
> ↓  
> 继续执行前文所述的零点补偿、角度环绕、舵角折叠与轮毂反转、  
> 转速换算及四轮统一等比例限速

# 12. 参考文献

[1] M.-H. Lee and T.-H. S. Li, “Kinematics, dynamics and control design of 4WIS4WID mobile robots,” *The Journal of Engineering*, vol. 2015, no. 1, pp. 6–16, 2015, DOI:  
[10.1049/joe.2014.0241] https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/joe.2014.0241

[2] 二维旋转矩阵与向量旋转 - 太阳与风 https://zhuanlan.zhihu.com/p/98007510

[3] 正交矩阵之旋转矩阵 - YourMath https://zhuanlan.zhihu.com/p/143056551

[4]【Lec2-1 刚体的构形和速度旋量（coordinate-free概念, 矢量/点/叉乘/旋转矩阵的意义和使用）】  
https://www.bilibili.com/video/BV1uY411G7AR/?  
share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb

[5]【中科大RM电控合集】各种底盘各种解算一网打尽  
https://www.bilibili.com/video/BV1toH6ekEfJ/?  
share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb

[6]【中科大RM电控合集】舵轮的适配算法 https://www.bilibili.com/video/BV1niNUe3EQB/?  
share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb

[7]【齐奇战队电控组培训:舵轮解算】 https://www.bilibili.com/video/BV13p4y177HZ/?  
share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb

[8]【机器人底盘全向模型详解】 https://www.bilibili.com/video/BV1f94y1W7AW/?  
share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb

[9]【_Dynamics_and_Aerial_Attitude_Control_for_Rapid_Emergenc舵轮底盘带运动学分析】  
https://www.bilibili.com/video/BV1fG411S7uN/?  
share_source=copy_web&vd_source=49d05f69dfc6663616a6cfbe19a35edb
