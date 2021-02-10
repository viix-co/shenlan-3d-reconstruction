# Kneip论文研读

Kneip是在已知$P_1,P_2,P_3$三个三维点和对应的二维点的情况下，求解相机外参($R$与$t$)的一种算法。

## 定义新相机座标系

原始相机座标系$\nu：(C,(1,0,0)^T,(0,1,0)^T,(0,0,1)^T)$。

由三点座标$P_1,P_2,P_3$做normalization得到三个单位向量$f_1,f_2,f_3$并定义新相机座标系$\tau：(C,t_x,t_y,t_z)$。

其中：

$\vec{t_x} = \vec{f_1}$

$\vec{t_z} = \frac{\vec{f_1} \times \vec{f_2}}{||\vec{f_1} \times \vec{f_2}||}$

$\vec{t_y} = \vec{t_z} \times \vec{t_x}$(因为$\vec{t_x}$与$\vec{t_z}$正交，所以这裡不需要做normalization)

$t_x,t_y,t_z$为$\tau$的basis vector在$\nu$裡的表达，它们组成的矩阵$T = [t_x,t_y,t_z]^T$即由$\nu$到$\tau$的旋转矩阵。

将点$f_i$从$\nu$转到$\tau$座标系：$f_i^\tau = T \cdot f_i$。

验证：$T \cdot t_x = [t_x,t_y,t_z]^T \cdot t_x = [1,0,0]^T$，即$t_x^\tau$($t_x$在$\tau$座标系下的表达)。

## 定义新世界座标系

原始世界座标系：$(O,X,Y,Z)$

新世界座标系$\eta = (P_1, n_x, n_y, n_z)$

其中：

$\vec{n_x} = \frac{\vec{P_1P_2}}{||\vec{P_1P_2}||}$

$\vec{n_z} = \frac{\vec{n_x} \times \vec{P_1P_3}}{||\vec{n_x} \times \vec{P_1P_3}||}$

$\vec{n_y} = \vec{n_z} \times \vec{n_x}$

由以上定义可以看出，$P_1,P_2,P_3$不共线$\eta$才可能存在。

由原始世界座标系转到新世界座标系$\eta$的旋转矩阵：$N = [n_x,n_y,n_z]^T$。

将点$P_i$由原始世界座标系转到新世界座标系$\eta$：$P_i^\eta = N \cdot (P_i-P_1)$。

## 定义平面$\Pi$座标系

平面$\Pi$座标系：$(P_1, t_x^\Pi, t_y^\Pi, t_z^\Pi)$。

### 与新相机座标系的关係

定义常数$d_{12} = ||P_1P_2||$。

从图上可以得知：常数$\cos\beta = f_1 \cdot f_2$及关係式$\frac{||CP_1||}{d_{12}} = \frac{sin(\pi-\alpha-\beta)}{sin\beta}$。

进一步定义常数$b = \cot \beta = \pm\sqrt{\frac{1}{1-\cos^2\beta}-1} = \pm\sqrt{\frac{1}{1-(f_1 \cdot f_2)^2}-1}$。

相机中心在平面$\Pi$的表达，为$\alpha = \angle P_2P_1C$的函数：

$C^\Pi(\alpha) \\= \begin{pmatrix}
\cos\alpha ||CP_1||\\
\sin\alpha ||CP_1||\\ 0
\end{pmatrix} \\= \begin{pmatrix}
\cos\alpha \cdot d_{12}\sin(\pi-\alpha-\beta)\sin^{-1}\beta\\
\sin\alpha \cdot d_{12}\sin(\pi-\alpha-\beta)\sin^{-1}\beta\\ 0
\end{pmatrix} \\= \begin{pmatrix}
d_{12}\cos\alpha(\sin\alpha \cot\beta + \cos\alpha)\\
d_{12}\sin\alpha(\sin\alpha \cot\beta + \cos\alpha)\\ 0
\end{pmatrix} \\= \begin{pmatrix}
d_{12}\cos\alpha(\sin\alpha \cdot b + \cos\alpha)\\
d_{12}\sin\alpha(\sin\alpha \cdot b + \cos\alpha)\\ 0
\end{pmatrix}$

$\tau$的basis vector$t_x^{\Pi}, t_y^{\Pi}, t_z^{\Pi}$在$\Pi$裡的表达：

$t_x^\Pi = (-\cos \alpha, -\sin \alpha, 0)^T$

$t_y^\Pi = (\sin \alpha, -\cos \alpha, 0)^T$

$t_z^\Pi = (0, 0, 1)^T$

从平面座标系$\Pi$到新相机座标系$\tau$的旋转矩阵为$[t_x^\pi, t_y^\pi, t_z^\pi]^T$。

从新相机座标系$\tau$到平面$\Pi$座标系的旋转矩阵$[t_x^\pi, t_y^\pi, t_z^\pi]$。

### 与新世界座标系的关係

$\Pi$与$\eta$座标系的原点重合。

从平面$\Pi$座标系到新世界座标系$\eta$的旋转矩阵为$R_\theta$，即沿$n_x$轴旋转$\theta$：

$R_\theta = \begin{pmatrix}
1 & 0 & 0\\
0 & \cos \theta & -\sin \theta \\
0 & \sin \theta & \cos \theta
\end{pmatrix}$

## 新世界座标系到新相机座标系

相机中心在新世界座标系$\eta$的表达，为$\alpha$及$\theta$的函数：

$C^\eta(\alpha, \theta) \\= R_\theta \cdot C^\Pi \\= 
\begin{pmatrix}
1 & 0 & 0\\
0 & \cos \theta & -\sin \theta \\
0 & \sin \theta & \cos \theta
\end{pmatrix} \cdot \begin{pmatrix}
d_{12}\cos\alpha(\sin\alpha \cdot b + \cos\alpha)\\
d_{12}\sin\alpha(\sin\alpha \cdot b + \cos\alpha)\\ 0
\end{pmatrix} \\=
\begin{pmatrix}
d_{12}\cos\alpha(\sin\alpha \cdot b + \cos\alpha)\\
d_{12}\sin\alpha\cos\theta(\sin\alpha \cdot b + \cos\alpha)\\
d_{12}\sin\alpha\sin\theta(\sin\alpha \cdot b + \cos\alpha)\\
\end{pmatrix}
$

从新世界座标系$\eta$到新相机座标系$\tau$的旋转矩阵=(新相机到新世界)^T=(平面$\Pi$到新世界 * 新相机到平面$\Pi$)^T，写成数学式如下：

$Q(\alpha, \theta) \\= 
[R_\theta \cdot (t_x^\Pi t_y^\Pi t_z^\Pi)]^T \\= 
[\begin{pmatrix}
1 & 0 & 0\\
0 & \cos \theta & -\sin \theta \\
0 & \sin \theta & \cos \theta
\end{pmatrix} \cdot
\begin{pmatrix}
-\cos \alpha & \sin \alpha & 0\\
-\sin \alpha & -\cos \alpha & 0 \\
0 & 0 & 1
\end{pmatrix}]^T \\= 
\begin{pmatrix}
-\cos \alpha & \sin \alpha & 0\\
-\sin \alpha \cos \theta & -\cos \alpha \cos \theta & - \sin\theta \\
-\sin \alpha \sin\theta & -\cos \alpha \sin \theta & \cos \theta
\end{pmatrix}^T \\= 
\begin{pmatrix}
-\cos \alpha & -\sin \alpha \cos \theta & -\sin \alpha \sin\theta\\
\sin \alpha & -\cos \alpha \cos \theta & -\cos \alpha \sin \theta \\
0 & - \sin\theta & \cos \theta
\end{pmatrix}$

将点$P_i$由新世界座标系$\eta$转到新相机座标系$\tau$：$P_i^\eta = Q(\alpha, \theta) \cdot (P_i-C^\eta(\alpha, \theta))$。

## 求解$\alpha$及$\theta$

到目前为止未知的数有$\alpha$及$\theta$，这裡利用$P_3^\tau$来列出关于$\cos \theta$的四次方程并求解。

因为$P_3^\eta$在$\eta$的xy平面上，所以可以将$P_3^\eta$表示为$(p_1,p_2,0)$。

欲求$P_3^\tau$，套用上面由新世界座标系的$P_3^\eta$到新相机座标系的公式：

$P_3^\tau \\= 
Q(\alpha, \theta) \cdot (P_3^\eta-C^\eta(\alpha,\theta)) \\= 
\begin{pmatrix}
-\cos \alpha & -\sin \alpha \cos \theta & -\sin \alpha \sin\theta\\
\sin \alpha & -\cos \alpha \cos \theta & -\cos \alpha \sin \theta \\
0 & - \sin\theta & \cos \theta
\end{pmatrix} \cdot (\begin{pmatrix}
p_1\\
p_2\\
0\\
\end{pmatrix}-\begin{pmatrix}
d_{12}\cos\alpha(\sin\alpha \cdot b + \cos\alpha)\\
d_{12}\sin\alpha\cos\theta(\sin\alpha \cdot b + \cos\alpha)\\
d_{12}\sin\alpha\sin\theta(\sin\alpha \cdot b + \cos\alpha)\\
\end{pmatrix}) \\= 
\begin{pmatrix}
-\cos \alpha \cdot p_1 -\sin \alpha \cos \theta \cdot p_2 + d_{12} (\sin\alpha \cdot b + \cos\alpha)(\cos^2\alpha + \sin^2\alpha \cos^2\theta + \sin^2\alpha \sin^2\theta) \\
\sin \alpha \cdot p_1 -\cos \alpha \cos \theta \cdot p_2 + d_{12} (\sin\alpha \cdot b + \cos\alpha)(-\sin\alpha \cos \alpha + \sin\alpha \cos \alpha \cos^2 \theta + \sin\alpha \cos \alpha \sin^2 \theta) \\
-\sin \theta \cdot p_2 + d_{12} (\sin\alpha \cdot b + \cos\alpha)(\sin\alpha \sin \theta \cos \theta - \sin\alpha \sin \theta \cos \theta)
\end{pmatrix} \\= 
\begin{pmatrix}
-\cos \alpha \cdot p_1 -\sin \alpha \cos \theta \cdot p_2 + d_{12} (\sin\alpha \cdot b + \cos\alpha) \\
\sin \alpha \cdot p_1 -\cos \alpha \cos \theta \cdot p_2 \\
-\sin \theta \cdot p_2 
\end{pmatrix}$

相机座标系下$P_3^\tau$经过缩放变成$f_3^\tau$($P_3^\tau$与$f_3^\tau$的方向需相同)。

首先定义两个常数$\phi_1 = \frac{f^\tau_{3,x}}{f^\tau_{3,z}}$及$\phi_2 = \frac{f^\tau_{3,y}}{f^\tau_{3,z}}$，并有以下关係：

$\begin{cases}
    \phi_1 = \frac{P^\tau_{3,x}}{P^\tau_{3,z}} \\
    \phi_2 = \frac{P^\tau_{3,y}}{P^\tau_{3,z}}
\end{cases} \\ \Leftrightarrow
\begin{cases}
    \phi_1 = \frac{-\cos \alpha \cdot p_1 -\sin \alpha \cos \theta \cdot p_2 + d_{12} (\sin\alpha \cdot b + \cos\alpha)}{-\sin \theta \cdot p_2 } \\
    \phi_2 = \frac{\sin \alpha \cdot p_1 -\cos \alpha \cos \theta \cdot p_2}{-\sin \theta \cdot p_2 }
\end{cases} \\ \Leftrightarrow
\begin{cases}
    \frac{\sin \theta}{\sin \alpha}p_2 = \frac{-\cot \alpha \cdot p_1 - \cos \theta \cdot p_2 + d_{12} (b + \cot\alpha)}{-\phi_1} \\
    \frac{\sin \theta}{\sin \alpha}p_2 = \frac{p_1 -\cot \alpha \cos \theta \cdot p_2}{-\phi_2}
\end{cases} \\ \Rightarrow
\begin{cases}
    \frac{\sin \theta}{\sin \alpha}p_2 = \frac{-\cot \alpha \cdot p_1 - \cos \theta \cdot p_2 + d_{12} (b + \cot\alpha)}{-\phi_1} \\
    \frac{\sin \theta}{\sin \alpha}p_2 = \frac{p_1 -\cot \alpha \cos \theta \cdot p_2}{-\phi_2}
\end{cases} \\ \Rightarrow
\phi_2(\cot\alpha+\cos\theta \cdot p_2 -d_{12}b - d_{12}\cot \alpha) = 
\phi_1(-p_1 + \cot \alpha \cos \theta \cdot p_2) \\ \Rightarrow 
(\phi_2(p_1-d_{12}) - \cos\theta p_2 \phi_1)\cot \alpha = -p_1\phi_1-\phi_2(\cos\theta \cdot p_2 - d_{12}b)) \\ \Rightarrow 
\cot \alpha = \frac{p_1\phi_1+\phi_2(\cos\theta \cdot p_2 - d_{12}b))}{\cos\theta p_2 \phi_1 - (\phi_2(p_1-d_{12})} = \frac{\frac{\phi_1}{\phi_2}p_1 + \cos \theta \cdot p_2 - d_{12}b}{\frac{\phi_1}{\phi_2} \cos \theta_1 p_2 - p_1 + d_{12}}$

到了这裡我们得到了以已知数$\phi_1$,$\phi_2$及未知数$\cos \theta$表达的$\cot \alpha$。

$\begin{equation}\begin{aligned}\phi_2 = \frac{P^\tau_{3,y}}{P^\tau_{3,z}} &\Leftrightarrow 
\phi_2 = \frac{\sin \alpha \cdot p_1 -\cos \alpha \cos \theta \cdot p_2}{-\sin \theta \cdot p_2} \\ &\Leftrightarrow 
(-\sin \theta p_2 \phi_2)^2 = (\sin \alpha \cdot p_1 -\cos \alpha \cos \theta \cdot p_2)^2 \\ &\Leftrightarrow 
\sin^2 \theta p_2^2 \phi_2^2 = \sin^2 \alpha(p_1 -\cot \alpha \cos \theta \cdot p_2)^2 \\ &\Leftrightarrow 
\sin^2 \theta \sec^2 \alpha p_2^2 \phi_2^2 = (p_1 -\cot \alpha \cos \theta \cdot p_2)^2 \\ &\Leftrightarrow 
(1-\cos^2 \theta) (1+\cot^2 \alpha) \phi_2^2 p_2^2 = p_1^2 -2\cot \alpha \cos \theta \cdot p_1p_2 + \cot^2 \alpha \cos^2 \theta \cdot p_2^2\end{aligned}\end{equation}$

得到$\cos\theta$和$\cot\alpha$的关係后，把公式(9)代入公式(10)，就可以列出$\cos\theta$的四次方程式：

$a_4 \cdot \cos^4\theta + a_3 \cdot \cos^3\theta + a_2 \cdot \cos^2\theta + a_1 \cdot \cos\theta + a_0 = 0$

接着求出最多四个$\cos\theta$的解，每个$\cos\theta$对应一个$\cot\alpha$。

然后由$\cos\theta$及$\cot\alpha$计算出$\theta$及$\alpha$。

## 求解R和C

得到了$\theta$及$\alpha$后，依公式(5)及(6)计算出$C^\eta(\alpha,\theta)$及$Q(\alpha,\theta)$。

相机中心在原始世界座标系裡的表达可由公式(2)的反运算得到：

$C = P_1 + N^T \cdot C^\eta$

原始相机座标系$\nu$到原始世界座标系的旋转矩阵=新世界座标系$\eta$到原世界座标系 * 新相机座标系$\tau$到新世界座标系$\eta$ * 原始相机座标系$\nu$到新相机座标系$\tau$，写成数学式如下：

$R = N^T \cdot Q^T \cdot T$

将$C^\eta$代入公式(12)可得$C$，将$Q$代入公式(13)可得$R$，至此我们已经求出了相机的四组外参。

通过对第四点重投影，我们可以选择其中误差最小的来当作最终的解。