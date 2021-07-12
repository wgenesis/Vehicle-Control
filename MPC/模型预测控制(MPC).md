# 模型预测控制(MPC)

##     	运动学模型

​        下图为简化的车辆运动学模型：



<img src="https://github.com/WeiHYDavid/Vehicle-Control/blob/main/images/KinematicModel.png" alt="运动学模型" style="zoom: 67%;" />

​		首先建立车辆的运动学模型，选取车辆的状态量和控制量：
$$
\chi =\left[ \begin{array}{c}	X\\	Y\\	\varphi\\\end{array} \right] \,\,  u=\left[ \begin{array}{c}	\nu _r\\	\delta _f\\\end{array} \right] \tag{2.1}
$$
其中状态量$X,Y,\varphi$分别为车辆的横坐标，纵坐标，航向角，控制量$v _r,\delta _f$分别为车辆的后轮速度和前轮转角。

​	   接下来推算车辆的运动学模型，其中状态量$X,Y$的一阶微分表达式由上图可得以下关系:
$$
\text{车辆}横\text{向速度：}\dot{X}=\nu _r\cos \varphi \tag{2.2}
$$

$$
\text{车辆纵向速度：}\dot{Y}=\nu _r\sin \varphi \tag{2.3}
$$

为推算状态量$\varphi$的一阶微分，将车辆的模型进一步简化：

<img src="https://github.com/WeiHYDavid/Vehicle-Control/blob/main/images/SimplifiedKinematicModel.png" style="zoom: 25%;" />

由上图的几何关系可得车辆旋转的线速度：
$$
\nu _{fy}=\nu _f\sin \delta _f \tag{2.4}
$$
其中：
$$
\nu _f=\frac{\nu _r}{\cos \delta _f} \tag{2.5}
$$
由圆的线速度和角速度的关系可得：
$$
\dot{\varphi}=\frac{\nu _{fy}}{l} \tag{2.6}
$$
联立式(2.4)、(2.5)、(2.6)可得状态量$\varphi$的一阶微分表达式为：
$$
\text{车辆}横\text{摆角速度：}\dot{\varphi}=\frac{\nu _r\tan \delta _f}{l} \tag{2.7}
$$
​		综上可得车辆的运行学模型：
$$
\dot{\chi}=f\left( \chi ,u \right) \tag{2.8}
$$
矩阵形式：
$$
\\\left[ \begin{array}{c}	\dot{X}_r\\	\dot{Y}_r\\	\dot{\varphi}\\\end{array} \right] =\left[ \begin{array}{c}	\cos \varphi\\	\sin \varphi\\	\tan \delta _f/l\\\end{array} \right] \nu _r \tag{2.9}
$$

## 	  线性化

​		由上面得到的运动学模型是一个非线性的模型，在后续优化求解的时候比较困难，因此需要先对式(2.9)进行线性化处理，这里采用的线性化方法为Taylor展开法。首先，设r点为当前时刻车辆跟踪的目标点，将向量方程(2.8)于目标点r处一阶Taylor展开：
$$
\dot{\chi}\approx f\left( \chi _r,u_r \right) +\frac{\partial f}{\partial \chi}^T\Bigg|_{\begin{array}{c}	\chi =\chi _r\\	u=u_r\\\end{array}}^{}\left( \chi -\chi _r \right) +\frac{\partial f}{\partial u}^T\Bigg|_{\begin{array}{c}	\chi =\chi _r\\	u=u_r\\\end{array}}^{}\left( u-u_r \right)\tag{2.10} 
$$
将式(2.10)减去式(2.8)：
$$
f\left( \chi ,u \right)-f\left( \chi _r,u_r \right) =\frac{\partial f}{\partial \chi}^T\Bigg|_{\begin{array}{c}	\chi =\chi _r\\	u=u_r\\\end{array}}^{}\left( \chi -\chi _r \right) +\frac{\partial f}{\partial u}^T\Bigg|_{\begin{array}{c}	\chi =\chi _r\\	u=u_r\\\end{array}}^{}\left( u-u_r \right)\tag{2.11}
$$
令$\tilde{\chi}=\chi -\chi _r,\tilde{u}=u -u _r$，并简写成以下形式：
$$
\dot{\tilde{\chi}}=a\tilde{\chi}+b\tilde{u}\tag{2.12}
$$
其中矩阵$a,b$为向量方程$f\left( \chi ,u \right)$分别对状态量$\chi$和控制量$u$求导的Jacobi矩阵，其形式为：
$$
a=\frac{\partial f}{\partial \chi}\Bigg|_{\begin{array}{c}	\chi =\chi _r\\	u=u_r\\\end{array}}^{}=\left[ \begin{matrix}	\frac{\partial f_1}{\partial \chi _1}&		\frac{\partial f_1}{\partial \chi _2}&		\frac{\partial f_1}{\partial \chi _3}\\	\frac{\partial f_2}{\partial \chi _1}&		\frac{\partial f_2}{\partial \chi _2}&		\frac{\partial f_2}{\partial \chi _3}\\	\frac{\partial f_3}{\partial \chi _1}&		\frac{\partial f_3}{\partial \chi _2}&		\frac{\partial f_3}{\partial \chi _3}\\\end{matrix} \right] =\left[ \begin{matrix}	0&		0&		-\nu _r\sin \varphi _r\\	0&		0&		\nu _r\cos \varphi _r\\	0&		0&		0\\\end{matrix} \right]\tag{2.13}
$$

$$
b=\frac{\partial f}{\partial u}\Bigg|_{\begin{array}{c}	\chi =\chi _r\\	u=u_r\\\end{array}}^{}=\left[ \begin{matrix}	\frac{\partial f_1}{\partial u_1}&		\frac{\partial f_1}{\partial u_2}\\	\frac{\partial f_2}{\partial u_1}&		\frac{\partial f_2}{\partial u_2}\\	\frac{\partial f_3}{\partial u_1}&		\frac{\partial f_3}{\partial u_2}\\\end{matrix} \right] =\left[ \begin{matrix}	\cos \varphi _r&		0\\	\sin \varphi _r&		0\\	\frac{\tan \delta _r}{l}&		\frac{\nu _r}{l\cos ^2\delta _r}\\\end{matrix} \right] \tag{2.14}
$$

## 	离散化

​		在上一节中完成了对状态方程的线性化处理，为使用MPC进行滚动优化，接下来还需要对式(2.12)进行离散化处理，设采样周期为时间$T$，根据前向欧拉法：
$$
\frac{\tilde{\chi}\left( k+1 \right) -\tilde{\chi}\left( k \right)}{T}=a\tilde{\chi}\left(k\right)+b\tilde{u}\left(k\right) \tag{2.15}
$$
对上式移项：
$$
\tilde{\chi}\left( k+1 \right) =\left( aT+I \right) \tilde{\chi}\left( k \right) +bT\tilde{u}\left( k \right) =A\tilde{\chi}\left( k \right) +B\tilde{u}\left( k \right)\tag{2.16}
$$
其中：
$$
A=aT+I=\left[ \begin{matrix}	1&		0&		-T\nu _r\sin \varphi _r\\	0&		1&		T\nu _r\cos \varphi _r\\	0&		0&		1\\\end{matrix} \right]\tag{2.17}
$$

$$
B=bT=\left[ \begin{matrix}	T\cos \varphi _r&		0\\	T\sin \varphi _r&		0\\	T\frac{\tan \delta _f}{l}&		T\frac{\nu _r}{l\cos ^2\delta _f}\\\end{matrix} \right] \tag{2.18}
$$

## 	预测

​		在得到离散化的状态方程后即可对车辆未来时刻的状态量进行预测，设当前采样时刻为$k$，预测域为$N_P$，控制时域为$N_C$，则有：
$$
\tilde{\chi}\left( k+1 \right) =A\tilde{\chi}\left( k \right) +B\tilde{u}\left( k \right)\tag{2.19}
$$

$$
\tilde{\chi}\left( k+2 \right) =A\tilde{\chi}\left( k+1 \right) +B\tilde{u}\left( k+1 \right)\tag{2.20}
$$

$$
\tilde{\chi}\left( k+3 \right) =A\tilde{\chi}\left( k+2 \right) +B\tilde{u}\left( k+2 \right)\tag{2.21}
$$

$$
\vdots
$$

$$
\tilde{\chi}\left( k+N_P \right) =A\tilde{\chi}\left( k+N_P-1 \right) +B\tilde{u}\left( k+N_C \right)\tag{2.22}
$$

将式(2.19)带入式(2.20)，再将其带入式(2.21)，以此类推，可得到只与当前采样时刻$k$的状态量有关的预测时域$N_P$内的状态量：
$$
\begin{aligned}	\tilde{\chi}\left( k+1 \right) &=A\tilde{\chi}\left( k \right) +B\tilde{u}\left( k \right)\\	\tilde{\chi}\left( k+2 \right) &=A^2\tilde{\chi}\left( k \right) +AB\tilde{u}\left( k \right) +B\tilde{u}\left( k+1 \right)\\	\tilde{\chi}\left( k+3 \right) &=A^3\tilde{\chi}\left( k \right) +A^2B\tilde{u}\left( k \right) +AB\tilde{u}\left( k+1 \right) +B\tilde{u}\left( k+2 \right)\\	\,\,\vdots\\	\tilde{\chi}\left( k+N_C \right) &=A^{N_C}\tilde{\chi}\left( k+N_C \right) +A^{N_C-1}B\tilde{u}\left( k \right) +A^{N_C-2}B\tilde{u}\left( k+1 \right) \cdots AB\tilde{u}\left( k+N_C-1 \right) +B\tilde{u}\left( k+N_C \right)\\	\vdots\\	\tilde{\chi}\left( k+N_P \right) &=A^{N_P}\tilde{\chi}\left( k+N_P \right) +A^{N_P-1}B\tilde{u}\left( k \right) +A^{N_P-2}B\tilde{u}\left( k+1 \right) \cdots A^{N_P-N_C-2}B\tilde{u}\left( k+N_C-1 \right) +A^{N_P-N_C-1}B\tilde{u}\left( k+N_C \right)\\\end{aligned}\tag{2.23}
$$
紧凑形式：
$$
Y=\varPsi \tilde{\chi}\left( k \right) +\varTheta  \tilde U\tag{2.24}
$$
其中：
$$
\begin{align}Y&=\left[ \begin{array}{c}	\tilde{\chi}\left( k+1 \right)\\	\tilde{\chi}\left( k+2 \right)\\	\vdots\\	\tilde{\chi}\left( k+N_P \right)\\\end{array} \right] \tag{2.25}\\\varPsi &=\left[ \begin{array}{l}	A\\	A^2\\	\vdots\\	A^{N_P}\\\end{array} \right] \,\,\tag{2.26}\\\tilde{\chi}(k)&=\left[ \begin{array}{c}	\tilde{X}(k)\\	\tilde{Y}(k)\\	\tilde{\varphi}(k)\\\end{array} \right] =\left[ \begin{array}{c}	X(k)-X_r(k)\\	Y(k)-Y_r(k)\\	\varphi(k) -\varphi _r(k)\\\end{array} \right]  \tag{2.27}\\\varTheta &=\left[ \begin{matrix}	B&		0&		\cdots&		0\\	AB&		B&		\cdots&		0\\	\vdots&		\vdots&		\ddots&		\vdots\\	A^{N_C-1}B&		A^{N_C-2}B&		\cdots&		B\\	A^{N_C}B&		A^{N_C-1}B&		\cdots&		AB\\	\vdots&		\vdots&		\ddots&		\vdots\\	A^{N_P-1}B&		A^{N_P-2}B&		\cdots&		A^{N_P-N_C-1}B\\\end{matrix} \right] \tag{2.28}\\\tilde U &=\left[ \begin{array}{c}	\tilde{u}\left( k \right)\\	\tilde{u}\left( k+1 \right)\\	\vdots\\	\tilde{u}\left( k+N_C \right)\\\end{array} \right] \tag{2.29}\end{align}
$$

## 优化		

​		接下来利用预测的未来时刻状态量和目标状态量构建损失函数，并优化求解得到未来控制时域内的控制序列。首先制定优化指标：
$$
\begin{align}\text{状态量误差尽可能小}&：Y=\tilde \chi=\chi-\chi_r\rightarrow 0\\\,\,      \text{控制量输出尽可能小}&： \tilde U\rightarrow 0\end{align}
$$
构建如下优化函数：
$$
J\left( k \right) =\sum_{j=1}^{N_P}{Y^T\left( k+j \right) QY\left( k+j \right)}+\sum_{j=0}^{N_C}{\tilde{u}\left( k+j \right) R\tilde{u}\left( k+j \right)}\tag{2.30}
$$
其中等式右侧第一项反映了系统对参考轨迹的跟踪能力，第二项要求输出控制量尽可能小，反映了对控制量平稳变化的控制要求，Q和R是状态量误差和控制量的权重矩阵，用于调整系统对状态量误差和控制量输出的敏感度。式(2.30)的2-范数形式：
$$
J\left( k \right) =\sum_{j=1}^{N_P}{\lVert Y \rVert}_{Q}^{2}+\sum_{j=1}^{N_C}{\lVert \tilde{u}\left( k+j \right) \rVert}_{R}^{2}\tag{2.31}
$$
矩阵形式：
$$
J=Y^TQ_QY+\tilde U^TR_R\tilde U \tag{2.32}
$$
为使用QP解算器，需要把式(2.31)转化成如式(2.33)的二次规范型：
$$
J=\frac{1}{2}x^THx+f^Tx\tag{2.33}
$$
将式(2.24)带入式(2.32)，并令$E=\varPsi \tilde{\chi}\left( k \right) $：
$$
\begin{align}J&=Y^TQ_QY+\tilde U^TR_R\tilde U \\&=\left[ E+\varTheta \tilde{U} \right] ^TQ_Q\left[ E+\varTheta \tilde{U} \right] +\tilde{U}^TR_R\tilde{U}\\&=E^TQE+\tilde{U}^T\varTheta ^TQ_QE+E^TQ_Q\varTheta \tilde{U}+\tilde{U}^T\left( \varTheta ^TQ_Q\varTheta +R \right) \tilde{U}\end{align}\tag{2.34}
$$
式(2.34)等式右侧第一项$E^EQE$与控制量$\tilde U$无关，可以去除，项$\tilde U^T\varTheta^T Q_Q E$与项$E^T Q_Q\varTheta\tilde U$同维，可以合并，简化为式(2.35)：
$$
\begin{align} J&=\tilde U^T(\varTheta^T Q_Q \varTheta + R_R)\tilde U +2E^TQ_Q\varTheta\tilde U\\&=2(\frac{1}{2}\tilde U^TH\tilde U+f^T\tilde U)\end{align}\tag{2.35}
$$

## 	更进一步

​		在式(2.12)中，采用的控制量为当前时刻控制量，这样存在明显的缺陷：无法对每个控制周期的控制增量进行约束，即无法避免被控系统控制量突变的现象，从而影响系统的连续性。因此采取新的状态量，将式(2.16)做如下变换：
$$
\tilde{\xi}\left( k \right) =\left[ \begin{array}{c}	\tilde{\chi}\left( k \right)\\	\tilde{u}\left( k-1 \right)\\\end{array} \right] =\left[ \begin{array}{c}	\tilde{X}\left( k \right)\\	\tilde{Y}\left( k \right)\\	\tilde{\varphi}\left( k \right)\\	\tilde{v}\left( k-1 \right)\\	\tilde{\delta}\left( k-1 \right)\\\end{array} \right] \tag{2.36}
$$
得到新的状态空间表达式：
$$
\begin{align}\left[ \begin{array}{c}	\tilde{\chi}\left( k+1 \right)\\	\tilde{u}\left( k \right)\\\end{array} \right] &=\left[ \begin{matrix}	A&		B\\	0_{m\times n}&		I_m\\\end{matrix} \right] \left[ \begin{array}{c}	\tilde{\chi}\left( k \right)\\	\tilde{u}\left( k-1 \right)\\\end{array} \right] +\left[ \begin{array}{c}	B\\	I_m\\\end{array} \right] \varDelta U\left( k \right)  \tag{2.37}\\\eta \left( k+1 \right) &=\left[ \begin{matrix}	I_{n}&		0\\\end{matrix} \right] \left[ \begin{array}{c}	\tilde{\chi}\left( k+1 \right)\\	\tilde{u}\left( k \right)\\\end{array} \right] \tag{2.38}\end{align}
$$
简写成：
$$
\begin{align}\tilde{\xi}\left( k+1 \right) &=\tilde{A}\tilde{\xi}\left( k \right) +\tilde{B}\varDelta U\left( k \right) \\\eta \left( k+1 \right) &=\tilde{C}\tilde{\xi}\left( k+1 \right) \end{align}\tag{2.39}
$$
其中$m$为控制量维度，$n$为系统输出量维度。由式(2.37)：

$$
\begin{align}\tilde{u}\left( k \right) &=\tilde{u}\left( k-1 \right) +\varDelta U\left( k \right) \\u\left( k \right) -u_r\left( k \right) &=u\left( k-1 \right) -u_r\left( k-1 \right) +\varDelta U\left( k \right)\\u\left( k \right)  &=u\left( k-1 \right) +u_r\left( k \right)-u_r\left( k-1 \right) +\varDelta U\left( k \right) \end{align}\tag{2.40}
$$
从而得到当前控制量与历史控制量的关系：
$$
\begin{align}u\left( k+1 \right) &=u\left( k \right) -u_r\left( k-1 \right) +u_r\left( k+1 \right) +\varDelta U\left( k \right) +\varDelta U\left( k+1 \right) \\u\left( k+N_C \right) &=u\left( k+N_C-1 \right) -u_r\left( k-1 \right) +u_r\left( k+N_C \right) +\sum_{j=0}^{N_C}{\varDelta u\left( k+j \right)}\end{align}\tag{2.41}
$$

在新的状态空间表达式(3.39)上预测未来预测时域内的系统状态量：
$$
\begin{aligned}	\eta \left( k+1 \right) &=\tilde{C}\tilde{A}\tilde{\xi}\left( k \right) +\tilde{C}\tilde{B}\varDelta U\left( k \right)\\	\eta \left( k+2 \right) &=\tilde{C}\tilde{A}^2\tilde{\xi}\left( k \right) +\tilde{C}AB\varDelta U\left( k \right) +\tilde{C}B\varDelta U\left( k+1 \right)\\	\eta \left( k+3 \right) &=\tilde{C}\tilde{A}^3\tilde{\xi}\left( k \right) +\tilde{C}\tilde{A}^2B\varDelta U\left( k \right) +\tilde{C}\tilde{A}\tilde{B}\varDelta U\left( k+1 \right) +\tilde{C}\tilde{B}\varDelta U\left( k+2 \right)\\	\,\,\vdots\\	\eta \left( k+N_C \right) &=\tilde{C}\tilde A^{N_C}\tilde{\xi}\left( k \right) +\tilde{C}\tilde A^{N_C-1}\tilde B\varDelta U\left( k \right) +\tilde{C}\tilde A^{N_C-2}\tilde B\varDelta U\left( k+1 \right) \cdots \tilde{C}\tilde A\tilde B\varDelta U\left( k+N_C-1 \right) +\tilde{C}\tilde B\varDelta U\left( k+N_C \right)\\\vdots\\\eta \left( k+N_P \right) &=\tilde{C}\tilde A^{N_P}\tilde{\xi}\left( k \right) +\tilde{C}\tilde A^{N_P-1}\tilde B\varDelta U\left( k \right) +\tilde{C}\tilde A^{N_P-2}\tilde B\varDelta U\left( k+1 \right) \cdots \tilde{C}\tilde A^{N_P-N_C-2}\tilde B\varDelta U\left( k+N_C-1 \right) +\tilde{C}\tilde{A}^{N_P-N_C-1}\tilde B\varDelta U\left( k+N_C \right)\\\end{aligned}\tag{2.42}
$$
紧凑形式：
$$
\tilde Y=\tilde\varPsi \tilde{\xi}\left( k \right) +\tilde\varTheta  \tilde U\tag{2.42}
$$
其中：
$$
\begin{aligned}	\tilde{Y}&=\left[ \begin{array}{c}	\tilde{\xi}\left( k+1 \right)\\	\tilde{\xi}\left( k+2 \right)\\	\vdots\\	\tilde{\xi}\left( k+N_P \right)\\\end{array} \right] \\	\tilde{\varPsi}&=\left[ \begin{array}{l}	C\tilde{A}\\	C\tilde{A}^2\\	\vdots\\	C\tilde{A}^{N_P}\\\end{array} \right] \,\,\\	\tilde{\xi}&=\left[ \begin{array}{c}	\tilde{X}\left( k \right)\\	\tilde{Y}\left( k \right)\\	\tilde{\varphi}\left( k \right)\\	\tilde{u}\left( k-1 \right)\\	\tilde{\varphi}\left( k-1 \right)\\\end{array} \right] =\left[ \begin{array}{c}	X\left( k \right) -X\left( k \right) _r\\	Y\left( k \right) -Y\left( k \right) _r\\	\varphi \left( k \right) -\varphi \left( k \right) _r\\	u\left( k-1 \right) -u_r\left( k-1 \right)\\	\varphi \left( k-1 \right) -\varphi _r\left( k-1 \right)\\\end{array} \right] \\	\tilde{\varTheta}&=\left[ \begin{matrix}	\tilde{C}\tilde{B}&		0&		\cdots&		0\\	\tilde{C}\tilde{A}\tilde{B}&		\tilde{C}\tilde{B}&		\cdots&		0\\	\vdots&		\vdots&		\ddots&		\vdots\\	\tilde{C}\tilde{A}^{N_C-1}\tilde{B}&		\tilde{C}A^{N_C-2}\tilde{B}&		\cdots&		\tilde{C}\tilde{B}\\	\tilde{C}\tilde{A}^{N_C}\tilde{B}&		\tilde{C}\tilde{A}^{N_C-1}\tilde{B}&		\cdots&		\tilde{C}\tilde{A}\tilde{B}\\	\vdots&		\vdots&		\ddots&		\vdots\\	\tilde{C}\tilde{A}^{N_P-1}\tilde{B}&		\tilde{C}\tilde{A}^{N_P-2}\tilde{B}&		\cdots&		\tilde{C}\tilde{A}^{N_P-N_C-1}\tilde{B}\\\end{matrix} \right] \\	\varDelta U&=\left[ \begin{array}{c}	\varDelta u\left( k \right)\\	\varDelta u\left( k+1 \right)\\	\vdots\\	\varDelta u\left( k+N_C \right)\\\end{array} \right] \\\end{aligned}\tag{2.43}
$$
性能指标函数：
$$
J\left( k \right) =\sum_{j=1}^{N_P}{\lVert \eta(k+j) \rVert}_{Q}^{2}+\sum_{j=0}^{N_C}{\lVert \varDelta{u}\left( k+j \right) \rVert}_{R}^{2}\tag{2.44}
$$
矩阵形式：
$$
J=\tilde Y^TQ_Q\tilde Y+\varDelta  U^TR_R\varDelta U \tag{2.45}
$$
将式(2.42)带入式(2.45)，并化简成二次规范型：
$$
J=\frac{1}{2}x^THx+f^Tx\tag{2.46}
$$
其中$H=\tilde\varTheta^T Q_Q \tilde\varTheta + R_R$，$f=E^TQ_Q\varTheta\tilde U$，$E=\tilde\varPsi \tilde{\xi}\left( k \right)$，利用QP解算器对式(2.46)进行解算可以得到预测时域内的一系列控制量增量：
$$
\varDelta U_{t}^{*}=\left[ \varDelta u_{t}^{*},\varDelta u_{t+1}^{*},\varDelta u_{t+2}^{*},\cdots ,\varDelta u_{t+N_C-1}^{*} \right] \tag{2.47}
$$
由式(2.40)即可知当前时刻的控制量为：
$$
u_t=u_{t-1}+u_r-u_{r\left( t_0-1 \right)}+u_{r\left( t-1 \right)}+\sum{\varDelta u}+\varDelta u_{t}^{*} \tag{2.48}
$$
