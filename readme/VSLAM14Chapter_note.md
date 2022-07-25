# 视觉SLAM十四讲Note

## 第一讲 - 预备知识

### 章节划分

1. 预备知识
2. SLAM系统概述
   1. 模块组成
   2. 编程环境，IDE
3. 三维空间刚体运动
   1. 旋转矩阵，欧拉角，四元数，Eigen
4. 李群李代数，Sophus
5. 相机模型，OpenCV
6. 非线性优化
7. 特征点法的视觉里程计
8. 直接法的视觉里程计
9. 后端优化
10. 后端优化位姿图
11. 回环检测
12. 地图构建，
    1. 单目稠密深度图构建
    2. RGB-D 稠密地图构建
13. 双目视觉里程计框架

### 习题答案

https://blog.csdn.net/jiahao62/article/details/80655542

## 第二讲 - 初识SLAM

### 传感器分类

- 外部传感器：导轨，二维码，GPS，依赖于环境，直接物理量
- 内部传感器：激光传感器，相机，轮式编码器，IMU，间接物理量

### 相机分类

- 单目相机 Monocular
  - 结构简单，成本低，
  - 无法确定物体真实大小 - 尺度不确定性
- 双目相机 Stereo
  - 通过基线估计每个像素空间位置，基线距离越大，测量物体可以越远
  - 消耗计算资源
- 深度相机 RGB-D
  - 红外结构光或者ToF(Time-of-Flight)，向物体发射光并接受返回的光
  - 测量范围窄，噪声大，视野小，易受阳光干扰，无法测量透射材质等

### 经典SLAM框架

- 传感器信息读取 Sensor Data
- 前端视觉里程计 VO (Visual Odometry)
- 后端（非线性）优化 (Optimization)
- 回环检测 (Loop Closure Detection)
- 建图 (Mapping)

### 视觉里程计 VO

- 主要关于计算机视觉领域，图像特征提取与匹配
- 通过相邻帧间图像估计相机运动，并恢复场景的空间结构
- 累积漂移 Accumulating Drift
- 需要**后端优化**和**回环检测**
  - 回环检测：负责把“机器人回到原始位置”检测出来
  - 后端优化：根据回环检测，校正整个轨迹

### 后端优化 Optimization

- 主要关于滤波与非线性优化算法
- 主要处理SLAM过程中的噪声，估计整个系统的状态（包括机器人轨迹，地图）
- 最大后验概率估计 MAP(Maximum-a-Posteriori)

### 回环检测 Loop Closure Detection

- 主要解决 位置估计随时间漂移的问题
- 识别到过的场景的能力，比如通过判断图像间的相似性

### 建图 Mapping

- 地图类型：
  - 2D栅格地图，2D拓扑地图，3D点云地图，3D网格地图
  - 度量地图与拓扑地图
- 度量地图(Metric Map)：精确表示地图中物体的位置关系
  - 稀疏(Sparse)：用于定位，
    - 抽象，不需要表达所有物体，只需要路标 Landmark
  - 半稠密
  - 稠密(Dense)：用于导航，
    - 用Grid或者Voxel表示一个小块，
    - 小块含有空闲，占据，未知三种状态，以表示是否有物体
- 拓扑地图(Topological Map)：强调元素之间的关系，图(Graph)
  - 图由节点和边构成，只考虑节点间的连通性
  - 不擅长复杂结构的地图

### SLAM数学表达

- 数学表达

$$
\begin{align}
离散时刻&：t = 1, \cdots, K \\
位置点&：\boldsymbol{x}_1,\cdots, \boldsymbol{x}_2 \\
路标点&：\boldsymbol{y}_1,\cdots, \boldsymbol{y}_2 \\
运动方程&：\boldsymbol{x}_k = f(\boldsymbol{x}_{k-1},\boldsymbol{u}_k, \boldsymbol{w}_k), 前一刻位置状态\boldsymbol{x}_{k-1}根据此刻运动传感器数据\boldsymbol{u}_k以及噪声\boldsymbol{w}_k来推导此刻的位置状态\boldsymbol{x}_k \\
观测方程&：\boldsymbol{z}_{k,j} = f(\boldsymbol{y}_{j},\boldsymbol{x}_k, \boldsymbol{v}_{k,j}),在此刻位置\boldsymbol{x}_k看到某个路标点\boldsymbol{y}_{j}时产生观测噪声\boldsymbol{v}_{k,j}和观测数据\boldsymbol{z}_{k,j} \\
基本\mathrm{SLAM}问题&：当知道运动数据\boldsymbol{u}以及传感器数据\boldsymbol{z}时，如何求解定位问题(估计\boldsymbol{x})和建图问题(估计\boldsymbol{y})
\end{align}
$$

- SLAM问题分类
  - 按照**运动和观测方程是否线性**，**噪声是否服从高斯分布**
    - 线性高斯系统, Linear Gaussian LG系统
      - 最简单，无偏最优估计由卡尔曼滤波器 (Kalman Filter, KF) 求解
    - 线性非高斯系统
    - 非线性高斯系统
    - 非线性非高斯系统，Non-Linear Non-Gaussian NLNG系统
      - 扩展卡尔曼滤波器 (Extended Kalman, Filter EKF)
      - 非线性优化 (None Linear Optimization)
        - 图优化 (Graph Optimization)
        
        

## 第三讲 - 三维空间刚体运动

### 向量计算

- 向量内积，可以描述向量间的投影关系


$$
\boldsymbol{a\cdot b} = \boldsymbol{a}^\mathrm{T}\boldsymbol{b} = \sum_{i=1}^{3}a_ib_i = |\boldsymbol{a}||\boldsymbol{b}|\cos<\boldsymbol{a}, \boldsymbol{b}>
$$

-  向量外积，外积的结果是一个垂直于叉乘的两个向量的向量，大小为$|a||b|sin<a,b>$,是两个向量张成的四边形的有向面积
  $$
  \boldsymbol{a}\times\boldsymbol{b} = 
      \begin{Vmatrix} 
      \boldsymbol{e}_1 & \boldsymbol{e}_2 & \boldsymbol{e}_3 \\
      a_1 & a_2 & a_3 \\
      b_1 & b_2 & b_3 \\
      \end{Vmatrix} = 
      
      \begin{bmatrix}
      a_2 b_3 - a_3 b_2 \\
      a_3 b_1 - a_1 b_3 \\
      a_1 b_2 - a_2 b_1
      \end{bmatrix} = 
      
      \begin{bmatrix}
      0 & -a_3 & a_2 \\
      a_3 & 0 & -a_1 \\
      -a_2 & a_1 & 0
      \end{bmatrix}\boldsymbol{b} 	
      
      \xlongequal{\mathrm{def}} 
      
  \boldsymbol{a}^{\land}\boldsymbol{b} \\
  $$
  
- 反对称矩阵：
  $$
  \boldsymbol{a}^{\wedge} = 
  	\begin{bmatrix}
      0 & -a_3 & a_2 \\
      a_3 & 0 & -a_1 \\
      -a_2 & a_1 & 0
  	\end{bmatrix}
  $$

### 旋转矩阵$\boldsymbol{R}$

- 旋转矩阵是正交矩阵，其逆为自身的转置

- 旋转矩阵的行列式为1

- 旋转矩阵属于特殊正交群 SO(3) (Special Orthogonal Group)
  $$
  \mathrm{SO}(3) = \{\boldsymbol{R}\in\R^{3\times 3}|\boldsymbol{R}\boldsymbol{R}^\mathrm{T}=\boldsymbol{I}, \mathrm{det}(\boldsymbol{R}) = 1\} \\
  \boldsymbol{a}' = \boldsymbol{R}^{-1}\boldsymbol{a} = \boldsymbol{R}^{\mathrm{T}}\boldsymbol{a}
  $$
  

### 变换矩阵$\boldsymbol{T}$

- 变换矩阵属于特殊欧式群SE(3)
  $$
  \mathrm{SE}(3) =
  \{
  \boldsymbol{T} = 
  	\begin{bmatrix}
  		\boldsymbol{R} & \boldsymbol{t} \\
  		\boldsymbol{0} & 1
  	\end{bmatrix} 
  	\in \R^{4\times 4} | \boldsymbol{R}\in\mathrm{SO}(3), t\in \R^3
  \} \\
  
  \boldsymbol{b} = \boldsymbol{T}_1\boldsymbol{a}, \boldsymbol{c} = \boldsymbol{T}_2\boldsymbol{a} \Rightarrow \boldsymbol{c} = \boldsymbol{T}_2 \boldsymbol{T}_1 \boldsymbol{a}\\
  
  \boldsymbol{T}^{-1} = 
  	\begin{bmatrix}
  		\boldsymbol{R}^{\mathrm{T}} & -\boldsymbol{R}^{\mathrm{T}}\boldsymbol{t} \\
  		\boldsymbol{0} & 1
  	\end{bmatrix}
  $$

### 旋转向量$\theta \boldsymbol{n}$

- 任意旋转都可以用一个旋转轴和一个旋转角来刻画

- 旋转向量的方向与单位长度的旋转轴$\boldsymbol{n}$一致，长度等于旋转角$\theta$，则可以表示为$\theta \boldsymbol{n}$

- 罗德里格斯公式

  - 从旋转向量 $\theta \boldsymbol{n}$ 到旋转矩阵 $\boldsymbol{R}$

  $$
  \boldsymbol{R} = \cos(\theta )\boldsymbol{I} + (1-\cos(\theta))\boldsymbol{n}\boldsymbol{n}^{\mathrm{T}} + \sin(\theta)\boldsymbol{n}^{\land}
  $$

  - 从旋转矩阵 $\boldsymbol{R}$ 到旋转向量 $\theta \boldsymbol{n}$

  $$
  \theta = \arccos(\frac{(1+2\cos(\theta)) - 1}{2}) \\
  \boldsymbol{R}\boldsymbol{n} = \boldsymbol{n}, (\boldsymbol{R}-\bold{I})\boldsymbol{n} = \boldsymbol{0}
  $$

- 旋转向量的奇异性发生在转角$\theta$超过$2\pi$时产生周期性



### 欧拉角

- 使用三个分离的旋转角
- 常用的RPY(Roll-Pitch-Yaw)，即ZYX旋转，假设刚体正前方为X轴朝向
  - 绕Z旋转，偏航角Yaw
  - 绕新Y旋转，俯仰角Pitch
  - 绕新X旋转，翻滚角Roll
- 欧拉角有万向锁从而产生奇异性问题，俯仰角为$\pm90^{\circ}$ 时，第一次旋转与第三次旋转在宏观层面效果相同，使系统丢失一个自由度。



### 四元数$\boldsymbol{q}$

- 四元数没有奇异性，可以解决三维向量描述旋转时的奇异性问题

- 四元数基本定义
  $$
  \begin{align}
  \boldsymbol{q} &= q_0 + q_1\mathrm{i} + q_2\mathrm{j} + q_3\mathrm{k} \\
  \boldsymbol{q} &= [s, \boldsymbol{v}^{\mathrm{T}}], s=q_0\in\R,\boldsymbol{v}=[q_1, q_2, q_3]^{\mathrm{T}}\in\R^3 \\
  \end{align}
  $$

- 四元数运算

  - 加减
    $$
    \boldsymbol{q}_a\pm\boldsymbol{q}_b = [s_a\pm s_b,\boldsymbol{v}_a\pm\boldsymbol{v}_b]^{\mathrm{T}}
    $$

  - 乘法
    $$
    \boldsymbol{q}_a \boldsymbol{q}_b = [s_a s_b - \boldsymbol{v}_a^{\mathrm{T}}\boldsymbol{v}_b, s_a\boldsymbol{v}_b + s_b\boldsymbol{v}_a + \boldsymbol{v}_a\times \boldsymbol{v}_b]^{\mathrm{T}}
    $$

  - 模长
    $$
    \begin{align}
    \begin{Vmatrix}\boldsymbol{q}_a\end{Vmatrix} &= \sqrt{s_a^2 + x_a^2 + y_a^2 + z_a^2} \\
    
    \begin{Vmatrix}\boldsymbol{q}_a\boldsymbol{q}_b\end{Vmatrix} &= \begin{Vmatrix}\boldsymbol{q}_a\end{Vmatrix}\begin{Vmatrix}\boldsymbol{q}_b\end{Vmatrix}
    \end{align}
    $$

  - 共轭
    $$
    \boldsymbol{q}^{*} = [s_a, -\boldsymbol{v}_a]^{\mathrm{T}} \\
    \boldsymbol{q}^{*}\boldsymbol{q} = \boldsymbol{q}\boldsymbol{q}^{*} = [s_a^2 + \boldsymbol{v}^{
    \mathrm{T}}\boldsymbol{v}, \boldsymbol{0}]^{\mathrm{T}}
    $$
    
  - 逆
    $$
    \boldsymbol{q}^{-1} = \boldsymbol{q}^{*}/\begin{Vmatrix}\boldsymbol{q}\end{Vmatrix}^2 \\
    \boldsymbol{q}\boldsymbol{q}^{-1} = \boldsymbol{q}^{-1}\boldsymbol{q} = 1 \\
    (\boldsymbol{q}_a\boldsymbol{q}_b)^{-1} = \boldsymbol{q}_b^{-1}\boldsymbol{q}_a^{-1}
    $$
    
  - 数乘
    $$
    \mathrm{k}\boldsymbol{q} = [\mathrm{k}s,\mathrm{k}\boldsymbol{v}]^{\mathrm{T}}
    $$
  
- 四元数旋转

  - 空间三维点 $\boldsymbol{p} = [x,y,z]\in\R^{3}$ 经过旋转 $\boldsymbol{q}$ 变为 $\boldsymbol{p}'$
    $$
    \boldsymbol{p} = [0,x,y,z]^{\mathrm{T}} = [0, \boldsymbol{v}]^\mathrm{T} \\
    \boldsymbol{p}' = \boldsymbol{q}\boldsymbol{p}\boldsymbol{q}^{-1}
    $$

- 四元数转换

  - 四元数到旋转矩阵
    $$
    \boldsymbol{R} = \boldsymbol{v}\boldsymbol{v}^{\mathrm{T}} + s^2\boldsymbol{I} + 2s\boldsymbol{v}^{\wedge} + (\boldsymbol{v}^{\wedge})^{2} \\
    $$

  - 四元数到旋转向量

  $$
  \begin{align}
  \theta &= 2\arccos(s) \\
  [n_x, n_y, n_z]^{\mathrm{T}} &= [q_1, q_2, q_3]^{\mathrm{T}}/\sin(\frac{\theta}{2})
  \end{align}
  $$

### 相似、仿射、射影变换

- 欧式变换

  - 自由度：6

  - 不变性质：长度、夹角、体积

  - $$
    \boldsymbol{T} = 
    \begin{bmatrix}
    \boldsymbol{R} & \boldsymbol{t} \\
    \boldsymbol{0} & 1
    \end{bmatrix}
    $$

    

- 相似变换

  - 自由度：7

  - 不变性质：体积比

  - 特点：

    - 比欧式变换多一个自由度，允许物体均匀缩放
    - 相似变换的集合也叫做相似变换群Sim(3)

  - $$
    \boldsymbol{T} = 
    \begin{bmatrix}
    s\boldsymbol{R} & \boldsymbol{t} \\
    \boldsymbol{0} & 1
    \end{bmatrix}
    $$

    

- 仿射变换

  - 自由度：12

  - 不变性质：平行性、体积比

  - 特点：

    - 只要求 $\boldsymbol{A}$ 是可逆矩阵，不必是正交矩阵，
    - 仿射变换也叫做正交投影，经过仿射变换后，立方体不再是方的，但各个平面仍然是平行四边形

  - $$
    \boldsymbol{T} = 
    \begin{bmatrix}
    \boldsymbol{A} & \boldsymbol{t} \\
    \boldsymbol{0} & 1
    \end{bmatrix}
    $$

    

- 射影变换

  - 自由度：15

  - 不变性质：接触平面的相交与相切

  - 特点： 

    - 左上角是可逆矩阵 $\boldsymbol{A}$，右上角是平移向量 $\boldsymbol{t}$，左下角是缩放向量 $\boldsymbol{a}^{\mathrm{T}}$。
    - 当 $v\neq 0$ 时，可以对整个矩阵除以 $v$ 得到右下角为1的矩阵，否则得到右下角为0的矩阵
    - 从真实世界到相机照片的变换可以看成是一个射影变换

  - $$
    \boldsymbol{T} = 
    \begin{bmatrix}
    \boldsymbol{A} & \boldsymbol{t} \\
    \boldsymbol{a}^{\mathrm{T}} & v
    \end{bmatrix}
    $$

    

## 第四讲 - 李群与李代数

### 李群与李代数基础

#### 李群

##### 群定义

只有一个良好的运算的集合，称之为群
$$
\begin{align}
\mathrm{SO}(3) &= \{\boldsymbol{R}\in\R^{3\times 3}|\boldsymbol{R}\boldsymbol{R}^\mathrm{T}=\boldsymbol{I}, \mathrm{det}(\boldsymbol{R}) = 1\} \\
\mathrm{SE}(3) &=
\{
\boldsymbol{T} = 
	\begin{bmatrix}
		\boldsymbol{R} & \boldsymbol{t} \\
		\boldsymbol{0} & 1
	\end{bmatrix} 
	\in \R^{4\times 4} | \boldsymbol{R}\in\mathrm{SO}(3), t\in \R^3
\}

\end{align}
$$

$$
\begin{align}
\boldsymbol{R}_{1}+\boldsymbol{R}_{2} \notin \mathrm{SO}(3)&, \quad \boldsymbol{T}_{1}+\boldsymbol{T}_{2} \notin \mathrm{SE}(3) \\
\boldsymbol{R}_{1} \boldsymbol{R}_{2} \in \mathrm{SO}(3)&, \quad \boldsymbol{T}_{1} \boldsymbol{T}_{2} \in \mathrm{SE}(3)

\end{align}
$$

##### 群性质

1. 封闭性
   $$
   \forall a_{1}, a_{2} \in A, \quad a_{1} \cdot a_{2} \in A
   $$

2. 结合律
   $$
   \forall a_{1}, a_{2}, a_{3} \in A, \quad\left(a_{1} \cdot a_{2}\right) \cdot a_{3}=a_{1} \cdot\left(a_{2} \cdot a_{3}\right)
   $$

3. 幺元
   $$
   \exists a_{0} \in A, \quad \text { s.t. } \quad \forall a \in A, \quad a_{0} \cdot a=a \cdot a_{0}=a
   $$

4. 逆

$$
\forall a \in A, \quad \exists a^{-1} \in A, \quad \text { s.t. } \quad a \cdot a^{-1}=a_{0}
$$

一般常见的群：

- 一般线性群 $\text{GL}(n)$
- 特殊正交群 $\text{SO}(n)$
- 特殊欧式群 $\text{SE}(n)$

##### 李群定义

李群指具有连续（光滑）性质的群。$\text{SO}(3)$ 与 $\text{SE}(3)$ 在实数空间上是连续的，想象一个刚体能够连续地在空间中运动。



#### 李代数

##### 李代数的引出

$$
\boldsymbol{R}(t) = \text{exp}(\boldsymbol{\phi}_0^{\wedge}t),\quad
\boldsymbol{\phi}(t) \in \R^{3}
$$

1. 给定某时刻的 $\boldsymbol{R}$，可以求得一个 $\boldsymbol{\phi}$，它描述了 $\boldsymbol{R}$ 在局部的导数关系，$\boldsymbol{\phi}$ 为对应到 $\text{SO}(3)$ 上的李代数 $\mathfrak{s o}(3)$
2. 李代数的指数映射，对数映射
   1. 指数映射： $\boldsymbol{R} = \text{exp}(\boldsymbol{\phi}^{\wedge}),\quad \mathfrak{s o}(3) \rightarrow \text{SO}(3)$
   2. 对数映射： $\boldsymbol{\phi} = \ln (\boldsymbol{R})^{\vee},\quad \text{SO}(3) \rightarrow \mathfrak{s o}(3)$



##### 李代数的定义

每个李群都有与之对应的李代数。李代数描述了李群的局部性质，准确的说，是单位元附近的正切空间。一般的李代数定义如下：

- 李代数由一个集合 $\mathbb{V}$、一个数域 $\mathbb{F}$ 和一个二元运算 $[,]$ 组成。如果它们满足以下性质，则称 $(\mathbb{V}, \mathbb{F}, [,])$ 为一个李代数，记为 $\mathfrak{g}$:

  1. 封闭性
     $$
     \forall \boldsymbol{X}, \boldsymbol{Y} \in \mathbb{V},[\boldsymbol{X}, \boldsymbol{Y}] \in \mathbb{V}
     $$

  2. 双线性
     $$
     \forall \boldsymbol{X}, \boldsymbol{Y}, \boldsymbol{Z} \in \mathbb{V}, a, b \in \mathbb{F} \\
     \text{s. t.} \quad [a \boldsymbol{X}+b \boldsymbol{Y}, \boldsymbol{Z}]=a[\boldsymbol{X}, \boldsymbol{Z}]+b[\boldsymbol{Y}, \boldsymbol{Z}], \quad[\boldsymbol{Z}, a \boldsymbol{X}+b \boldsymbol{Y}]=a[\boldsymbol{Z}, \boldsymbol{X}]+b[\boldsymbol{Z}, \boldsymbol{Y}]
     $$

  3. 自反性
     $$
     \forall \boldsymbol{X} \in \mathbb{V},[\boldsymbol{X}, \boldsymbol{X}]=\mathbf{0}
     $$

  4. 雅可比等价
     $$
     \forall \boldsymbol{X}, \boldsymbol{Y}, \boldsymbol{Z} \in \mathbb{V},[\boldsymbol{X},[\boldsymbol{Y}, \boldsymbol{Z}]]+[\boldsymbol{Z},[\boldsymbol{X}, \boldsymbol{Y}]]+[\boldsymbol{Y},[\boldsymbol{Z}, \boldsymbol{X}]]=\mathbf{0}
     $$
     

##### 李代数 $\mathfrak{s o}(3)$

1. 李代数 $\mathfrak{s o}(3)$ 的反对称矩阵 $\boldsymbol{\Phi}$
   $$
   \boldsymbol{\Phi}= \boldsymbol{\phi}^{\wedge}=\left[\begin{array}{ccc}
   0 & -\phi_{3} & \phi_{2} \\
   \phi_{3} & 0 & -\phi_{1} \\
   -\phi_{2} & \phi_{1} & 0
   \end{array}\right] \in \mathbb{R}^{3 \times 3}
   $$

2. $\mathfrak{s o}(3)$ 的定义
   $$
   \mathfrak{s o}(3)=\left\{\boldsymbol{\phi} \in \mathbb{R}^{3}, \boldsymbol{\Phi}=\boldsymbol{\phi}^{\wedge} \in \mathbb{R}^{3 \times 3}\right\}
   $$

3.  $\mathfrak{s o}(3)$ 到  $\text{SO}(3)$ 的指数映射
   $$
   \boldsymbol{R}=\exp \left(\boldsymbol{\phi}^{\wedge}\right)
   $$

4. $\mathfrak{s o}(3)$ 的李括号
   $$
   \left[\boldsymbol{\phi}_{1}, \boldsymbol{\phi}_{2}\right]=\left(\boldsymbol{\Phi}_{1} \boldsymbol{\Phi}_{2}-\boldsymbol{\Phi}_{2} \boldsymbol{\Phi}_{1}\right)^{\vee}
   $$

##### 李代数 $\mathfrak{s e}(3)$

1. $\mathfrak{s e}(3)$ 的定义
   $$
   \mathfrak{s e}(3)=\left\{\boldsymbol{\xi}=\left[\begin{array}{c}
   \boldsymbol{\rho} \\
   \boldsymbol{\phi}
   \end{array}\right] \in \mathbb{R}^{6}, \boldsymbol{\rho} \in \mathbb{R}^{3}, \boldsymbol{\phi} \in \mathfrak{s o}(3), \boldsymbol{\xi}^{\wedge}=\left[\begin{array}{cc}
   \boldsymbol{\phi}^{\wedge} & \boldsymbol{\rho} \\
   \mathbf{0}^{\mathrm{T}} & 0
   \end{array}\right] \in \mathbb{R}^{4 \times 4}\right\}
   $$

2. $\mathfrak{s e}(3)$ 到 $\text{SO}(3)$ 的指数映射
   $$
   \boldsymbol{T}=\exp \left(\boldsymbol{\xi}^{\wedge}\right)
   $$

3. $\mathfrak{s e}(3)$ 的李括号
   $$
   \left[\boldsymbol{\xi}_{1}, \boldsymbol{\xi}_{2}\right]=\left(\boldsymbol{\xi}_{1}^{\wedge} \boldsymbol{\xi}_{2}^{\wedge}-\boldsymbol{\xi}_{2}^{\wedge} \boldsymbol{\xi}_{1}^{\wedge}\right)^{\vee}
   $$

### 指数与对数映射

#### $\mathfrak{s o}(3)$ 指数映射与对数映射

定义 $\boldsymbol{\phi}$ 的模长为 $\theta$，方向为 $\boldsymbol{a}$：
$$
\boldsymbol{\phi} = \theta \boldsymbol{a},\quad \|\boldsymbol{a}\|=1 \\
$$
 $\mathfrak{s o}(3)$ 到  $\text{SO}(3)$ 的指数映射：
$$
\begin{align}
\boldsymbol{R} 
&= \exp(\boldsymbol{\boldsymbol{\phi}^{\wedge}}) \\
&= \exp \left(\theta \boldsymbol{a}^{\wedge}\right)\\
&= \sum_{n=0}^{\infty} \frac{1}{n !}\left(\theta \boldsymbol{a}^{\wedge}\right)^{n} \\
&= \cos \theta \boldsymbol{I}+(1-\cos \theta) \boldsymbol{a} \boldsymbol{a}^{\mathrm{T}}+\sin \theta \boldsymbol{a}^{\wedge}
\end{align}
$$
$\text{SO}(3)$ 到 $\mathfrak{s o}(3)$ 的对数映射：
$$
\begin{align}
\boldsymbol{\phi}
&= \ln (\boldsymbol{R})^{\vee}=\left(\sum_{n=0}^{\infty} \frac{(-1)^{n}}{n+1}(\boldsymbol{R}-\boldsymbol{I})^{n+1}\right)^{\vee} \\
&= \theta \boldsymbol{a} \\
\theta &= \arccos(\frac{\mathrm{tr}(\boldsymbol{R}) - 1}{2}) \\
\boldsymbol{R}\boldsymbol{a} &= \boldsymbol{a},\quad (\boldsymbol{R} - \boldsymbol{I})\boldsymbol{a} = \boldsymbol{0}
\end{align}
$$

#### $\mathfrak{s e}(3)$ 指数映射与对数映射

 $\mathfrak{s e}(3)$ 到  $\text{SE}(3)$ 的指数映射：
$$
\begin{align}
\boldsymbol{T} 
&= \exp \left(\boldsymbol{\xi}^{\wedge}\right)
\\
&=\left[\begin{array}{cc}
\sum_{n=0}^{\infty} \frac{1}{n !}\left(\boldsymbol{\phi}^{\wedge}\right)^{n} & \sum_{n=0}^{\infty} \frac{1}{(n+1) !}\left(\boldsymbol{\phi}^{\wedge}\right)^{n} \boldsymbol{\rho} \\
\mathbf{0}^{\mathrm{T}} & 1
\end{array}\right] \\

&\triangleq\left[\begin{array}{cc}
\boldsymbol{R} & \boldsymbol{J} \boldsymbol{\rho} \\
\mathbf{0}^{\mathrm{T}} & 1
\end{array}\right]

\\
\\
\boldsymbol{\phi} &= \theta \boldsymbol{a},\quad \|\boldsymbol{a}\|=1,\quad 

\boldsymbol{\xi} =  
\left[\begin{array}{c}
\boldsymbol{\phi} \\
\boldsymbol{\rho}
\end{array}\right]

\\

\boldsymbol{R} 
&= \cos \theta \boldsymbol{I}+(1-\cos \theta) \boldsymbol{a} \boldsymbol{a}^{\mathrm{T}}+\sin \theta \boldsymbol{a}^{\wedge} \\



\boldsymbol{J}
&= \frac{\sin \theta}{\theta} \boldsymbol{I}+\left(1-\frac{\sin \theta}{\theta}\right) \boldsymbol{a} \boldsymbol{a}^{\mathrm{T}}+\frac{1-\cos \theta}{\theta} \boldsymbol{a}^{\wedge}\\

\boldsymbol{t} &=\boldsymbol{J}\boldsymbol{\rho}


\end{align}
$$
$\text{SE}(3)$ 到 $\mathfrak{s e}(3)$ 的对数映射：
$$
\begin{align}
\boldsymbol{\phi}
&= \theta \boldsymbol{a} \\


\theta &= \arccos(\frac{\mathrm{tr}(\boldsymbol{R}) - 1}{2}) \\
\boldsymbol{R}\boldsymbol{a} &= \boldsymbol{a},\quad (\boldsymbol{R} - \boldsymbol{I})\boldsymbol{a} = \boldsymbol{0} \\

\boldsymbol{J}\boldsymbol{\rho} &= \boldsymbol{t} \\

\boldsymbol{\xi} &=  
\left[\begin{array}{c}
\boldsymbol{\phi} \\
\boldsymbol{\rho}
\end{array}\right] 
= 
\left[\begin{array}{c}
\boldsymbol{\phi} \\
\boldsymbol{\theta \boldsymbol{a}}
\end{array}\right] 


\end{align}
$$

#### 三维旋转与三维变换指数映射与对数映射关系表 



<img src="VSLAM14Chapter_note.assets/image-20220530181448415-16539056969111.png" alt="image-20220530181448415" style="zoom: 33%;" />





### 李代数求导与扰动模型

#### BCH近似

在 $\text{SO}(3)$ 中的两个矩阵相乘，无法对应 $\mathfrak{so}(3)$ 中的两个李代数相加，因为对于矩阵来说，下式不成立:
$$
\ln (\exp (\boldsymbol{A}) \exp (\boldsymbol{B}))=\boldsymbol{A}+\boldsymbol{B}
$$
两个李代数指数映射**乘积**的完整形式如下(BCH公式), 其中$[\ ]$为李括号:
$$
\ln (\exp (\boldsymbol{A}) \exp (\boldsymbol{B}))=\boldsymbol{A}+\boldsymbol{B}+\frac{1}{2}[\boldsymbol{A}, \boldsymbol{B}]+\frac{1}{12}[\boldsymbol{A},[\boldsymbol{A}, \boldsymbol{B}]]-\frac{1}{12}[\boldsymbol{B},[\boldsymbol{A}, \boldsymbol{B}]]+\cdots
$$
考虑 $\text{SO}(3)$ 上的李代数 $\ln \left(\exp \left(\boldsymbol{\phi}_{1}^{\wedge}\right) \exp \left(\boldsymbol{\phi}_{2}^{\wedge}\right)\right)^{\vee}$, 当 $\boldsymbol{\phi}_1$ 或 $\boldsymbol{\phi}_2$  为小量时, 小量二次以上的项都可以被忽略, 此时BCH拥有线性近似表达:
$$
\ln \left(\exp \left(\boldsymbol{\phi}_{1}^{\wedge}\right) \exp \left(\boldsymbol{\phi}_{2}^{\wedge}\right)\right)^{\vee} \approx \begin{cases}\boldsymbol{J}_{l}\left(\boldsymbol{\boldsymbol{\phi}}_{2}\right)^{-1} \boldsymbol{\boldsymbol{\phi}}_{1}+\boldsymbol{\boldsymbol{\phi}}_{2} & \text { 当 } \boldsymbol{\phi}_{1} \text { 为小量, } \\ \boldsymbol{J}_{r}\left(\boldsymbol{\phi}_{1}\right)^{-1} \boldsymbol{\phi}_{2}+\boldsymbol{\phi}_{1} & \text { 当 } \boldsymbol{\phi}_{2} \text { 为小量. }\end{cases}
$$
![image-20220704222952603](VSLAM14Chapter_note.assets/image-20220704222952603.png)

BCH近似的意义：

- 对于 $\mathfrak{so}(3)$

  - 李群上的乘法对应李代数上的加法, 对于某个旋转 $\boldsymbol{R}$ , 对应的李代数为 $\boldsymbol{\phi}$。对它左乘一个微小旋转 $\Delta\boldsymbol{R}$, 对应李代数 $\Delta\boldsymbol{\phi}$, 在李群上得到结果 $\Delta\boldsymbol{R}\cdot\boldsymbol{R}$, 李代数上根据BCH近似，为$\boldsymbol{J}_l^{-1}(\boldsymbol{\phi})\Delta\boldsymbol{\phi}+\boldsymbol{\phi}$ , 如下：
    $$
    \exp \left(\Delta \boldsymbol{\phi}^{\wedge}\right) \exp \left(\boldsymbol{\phi}^{\wedge}\right)=\exp \left(\left(\boldsymbol{\phi}+J_{l}^{-1}(\boldsymbol{\phi}) \Delta \boldsymbol{\phi}\right)^{\wedge}\right)
    $$
    
  - 李代数上的加法对应李群上的乘法：
    $$
    \exp \left((\boldsymbol{\phi}+\Delta \boldsymbol{\phi})^{\wedge}\right)=\exp \left(\left(\boldsymbol{J}_{l} \Delta \boldsymbol{\phi}\right)^{\wedge}\right) \exp \left(\boldsymbol{\phi}^{\wedge}\right)=\exp \left(\boldsymbol{\phi}^{\wedge}\right) \exp \left(\left(\boldsymbol{J}_{r} \Delta \boldsymbol{\phi}\right)^{\wedge}\right)
    $$
  
- 对于$\mathfrak{se}(3)$
  $$
  \begin{aligned}
  &\exp \left(\Delta \boldsymbol{\xi}^{\wedge}\right) \exp \left(\boldsymbol{\xi}^{\wedge}\right) \approx \exp \left(\left(\boldsymbol{\mathcal{J}}_{l}^{-1} \Delta \boldsymbol{\xi}+\boldsymbol{\xi}\right)^{\wedge}\right) \\
  &\exp \left(\boldsymbol{\xi}^{\wedge}\right) \exp \left(\Delta \boldsymbol{\xi}^{\wedge}\right) \approx \exp \left(\left(\boldsymbol{\mathcal{J}}_{r}^{-1} \Delta \boldsymbol{\xi}+\boldsymbol{\xi}\right)^{\wedge}\right)
  \end{aligned}
  $$



#### $\text{SO}(3)$ 上的求导

为了优化位姿的估计值, 经常会讨论关于位姿函数的导数, 有以下两种方法对位姿函数的求导

-   李代数求导法: 用李代数表示姿态, 根据李代数加法进行李代数求导
-   扰动求导法: 对李群左乘或右乘一个微小扰动, 对该扰动求导

##### 李代数求导法

对空间点 $\boldsymbol{p}$ 旋转 $\boldsymbol{R}$, 得到 $\boldsymbol{R}\boldsymbol{p}$, 设 $\boldsymbol{R}$ 对应的李代数为 $\boldsymbol{\phi}$, 计算旋转之后点的坐标相对于旋转的导数:
$$
\begin{align}
\frac{\part(\boldsymbol{R}\boldsymbol{p})}{\part\boldsymbol{R}} = 
\frac{\partial\left(\exp \left(\boldsymbol{\phi}^{\wedge}\right) \boldsymbol{p}\right)}{\partial \boldsymbol{\phi}} = 
-(\boldsymbol{R} \boldsymbol{p})^{\wedge} \boldsymbol{J}_{l}
\end{align} 
$$

##### 扰动模型求导法(左乘)

对空间点 $\boldsymbol{p}$ 旋转 $\boldsymbol{R}$, 得到 $\boldsymbol{R}\boldsymbol{p}$, 对$\boldsymbol{R}$ 进行一次扰动 $\Delta\boldsymbol{R}$, 看结果相对于扰动的变化率. 以左扰动为例, 设左扰动 $\Delta\boldsymbol{R}$ 对应的李代数为 $\boldsymbol{\varphi}$, 对 $\boldsymbol{\varphi}$ 求导, 结果比李代数求导法省去一个 $\boldsymbol{J}_l$ 的计算: 
$$
\frac{\partial(\boldsymbol{R} \boldsymbol{p})}{\partial \boldsymbol{\varphi}}=\lim _{\boldsymbol{\varphi} \rightarrow 0} \frac{\exp \left(\boldsymbol{\varphi}^{\wedge}\right) \exp \left(\boldsymbol{\phi}^{\wedge}\right) \boldsymbol{p}-\exp \left(\boldsymbol{\phi}^{\wedge}\right) \boldsymbol{p}}{\boldsymbol{\varphi}} = -(\boldsymbol{R}\boldsymbol{p})^{\wedge}
$$



#### $\text{SE}(3)$ 上的求导

##### 扰动模型求导法(左乘)

假设某空间点 $\boldsymbol{p}$ 经过一次变换 $\boldsymbol{T}$ (对应李代数为$\boldsymbol{\xi}$), 得到 $\boldsymbol{T}\boldsymbol{p}$, 给 $\boldsymbol{T}$ 左乘一个扰动 $\Delta\boldsymbol{T} = \exp(\delta\boldsymbol{\xi}^{\wedge})$, 设扰动项的李代数为 $\delta\boldsymbol{\xi} = [\delta\boldsymbol{\rho}, \delta\boldsymbol{\phi}]^{\mathrm{T}}$, 那么:
$$
\begin{align}
\frac{\partial(\boldsymbol{T} \boldsymbol{p})}{\partial \delta \boldsymbol{\xi}} &= \lim _{\delta \boldsymbol{\xi} \rightarrow 0} \frac{\exp \left(\delta \boldsymbol{\xi}^{\wedge}\right) \exp \left(\boldsymbol{\xi}^{\wedge}\right) \boldsymbol{p}-\exp \left(\boldsymbol{\xi}^{\wedge}\right) \boldsymbol{p}}{\delta \boldsymbol{\xi}} \\

&= \left[\begin{array}{cc}
\boldsymbol{I} & -(\boldsymbol{R} \boldsymbol{p}+\boldsymbol{t})^{\wedge} \\
\mathbf{0}^{\mathrm{T}} & \mathbf{0}^{\mathrm{T}}
\end{array}\right] \stackrel{\text { def }}{=}(\boldsymbol{T} \boldsymbol{p})^{\odot}
\end{align}
$$


### 评估轨迹误差

-   绝对误差  (ATE, Absolute Trajectory Error)

    -   绝对轨迹误差 (ATE, Absolute Trajectory Error), 实际也是均方根误差 (RMSE, Root-Mean-Squared Error)
        $$
        \mathrm{ATE}_{\mathrm{all}}=\sqrt{\frac{1}{N} \sum_{i=1}^{N}\left\|\log \left(\boldsymbol{T}_{\mathrm{gt}, i}^{-1} \boldsymbol{T}_{\mathrm{esti}, i}\right)^{\vee}\right\|_{2}^{2}}
        $$
        
    -   平均平移误差 (ATE, Average Translational Error), $\text{trans}()$ 表示取括号内部变量的平移部分
        $$
        \mathrm{ATE}_{\text {trans }}=\sqrt{\frac{1}{N} \sum_{i=1}^{N}\left\|\operatorname{trans}\left(\boldsymbol{T}_{\mathrm{gt}, i}^{-1} \boldsymbol{T}_{\text {esti, } i}\right)\right\|_{2}^{2}}
        $$
        
    
-   相对误差 (RPE, Relative Pose Error)

    -   相对轨迹误差
        $$
        \mathrm{RPE}_{\mathrm{all}}=\sqrt{\frac{1}{N-\Delta t} \sum_{i=1}^{N-\Delta t} \| \log \left(\left(T_{\mathrm{gt}, i}^{-1} \boldsymbol{T}_{\mathrm{gt}, i+\Delta t}\right)^{-1}\left(\boldsymbol{T}_{\mathrm{est}, i}^{-1} \boldsymbol{T}_{\text {esti }, i+\Delta t}\right)\right)^{\vee} \|_{2}^{2}}
        $$
        
    -   相对平移误差
        $$
        \mathrm{RPE}_{\text {trans }}=\sqrt{\frac{1}{N-\Delta t} \sum_{i=1}^{N-\Delta t} \| \operatorname{trans}\left(\left(\boldsymbol{T}_{\mathrm{gt}, i}^{-1} \boldsymbol{T}_{\mathrm{gt}, i+\Delta t}\right)^{-1}\left(\boldsymbol{T}_{\mathrm{est}, i}^{-1} \boldsymbol{T}_{\mathrm{est}, i+\Delta t}\right)\right) \|_{2}^{2}}
        $$
        



### 相似变换群 $\text{Sim}(3)$ 与李代数

-   相似变换
    $$
    \boldsymbol{p}^{\prime}=\left[\begin{array}{cc}
    s \boldsymbol{R} & \boldsymbol{t} \\
    \mathbf{0}^{\mathrm{T}} & 1
    \end{array}\right] \boldsymbol{p}=s \boldsymbol{R} \boldsymbol{p}+\boldsymbol{t}
    $$
    
-   相似变换群 $\text{Sim}(3)$
    $$
    \operatorname{Sim}(3)=\left\{\boldsymbol{S}=\left[\begin{array}{cc}
    s \boldsymbol{R} & \boldsymbol{t} \\
    \mathbf{0}^{\mathrm{T}} & 1
    \end{array}\right] \in \mathbb{R}^{4 \times 4}\right\}
    $$
    
-   相似变换群的李代数 $\mathfrak{sim}(3)$
    $$
    \operatorname{sim}(3)=\left\{\boldsymbol{\zeta} \mid \boldsymbol{\zeta}=\left[\begin{array}{l}
    \boldsymbol{\rho} \\
    \boldsymbol{\phi} \\
    \sigma
    \end{array}\right] \in \mathbb{R}^{7}, \boldsymbol{\zeta}^{\wedge}=\left[\begin{array}{cc}
    \sigma \boldsymbol{I}+\boldsymbol{\phi}^{\wedge} & \boldsymbol{\rho} \\
    \mathbf{0}^{\mathrm{T}} & 0
    \end{array}\right] \in \mathbb{R}^{4 \times 4}\right\}
    $$
    
-   相似变换群的指数映射
    $$
    \begin{align}
    \exp \left(\boldsymbol{\zeta}^{\wedge}\right) =& \left[\begin{array}{cc}
    \mathrm{e}^{\sigma} \exp \left(\phi^{\wedge}\right) & \boldsymbol{J}_{s} \rho \\
    \mathbf{0}^{\mathrm{T}} & 1
    \end{array}\right],\\
    
    s =& \mathrm{e}^{\sigma}, 
    \boldsymbol{R}=\exp \left(\boldsymbol{\phi}^{\wedge}\right), \boldsymbol{t}=\boldsymbol{J}_{s} \boldsymbol{\rho}, \\
    
    
    \boldsymbol{J}_{s} =& \frac{\mathrm{e}^{\sigma}-1}{\sigma} \boldsymbol{I}+\frac{\sigma \mathrm{e}^{\sigma} \sin \theta+\left(1-\mathrm{e}^{\sigma} \cos \theta\right) \theta}{\sigma^{2}+\theta^{2}} \boldsymbol{a}^{\wedge} \\
    
    &+\left(\frac{\mathrm{e}^{\sigma}-1}{\sigma}-\frac{\left(\mathrm{e}^{\sigma} \cos \theta-1\right) \sigma+\left(\mathrm{e}^{\sigma} \sin \theta\right) \theta}{\sigma^{2}+\theta^{2}}\right) \boldsymbol{a}^{\wedge} \boldsymbol{a}^{\wedge} .
    
    
    
    \end{align}
    $$
    
-   相思变换群的扰动模型

    -   $\operatorname{Sim}(3)$ 的 BCH 近似与 $\text{SE}(3)$ 是类似的。我们可以讨论一个点 $\boldsymbol{p}$ 经过相似变换 $\boldsymbol{S} \boldsymbol{p}$ 后, 相对 于 $\boldsymbol{S}$ 的导数。同样地, 存在微分模型和扰动模型两种方式, 而扰动模型较为简单。我们省略推导 过程, 直接给出扰动模型的结果。设给予 $\boldsymbol{S} \boldsymbol{p}$ 左侧一个小扰动 $\exp \left(\boldsymbol{\zeta}^{\wedge}\right)$, 并求 $\boldsymbol{S} \boldsymbol{p}$ 对于扰动的导数。因为 $\boldsymbol{S p}$ 是 4 维的齐次坐标, $\boldsymbol{\zeta}$ 是 7 维向量, 所以该导数应该是 $4 \times 7$ 的雅可比。方便起见, 记 $\boldsymbol{S p}$ 的前 3 维组成向量为 $\boldsymbol{q}$, 那么:

$$
\frac{\partial \boldsymbol{S} \boldsymbol{p}}{\partial \boldsymbol{\zeta}}=\left[\begin{array}{ccc}
\boldsymbol{I} & -\boldsymbol{q}^{\wedge} & \boldsymbol{q} \\
\mathbf{0}^{\mathrm{T}} & \boldsymbol{0}^{\mathrm{T}} & 0
\end{array}\right]
$$



## 第五讲 - 相机与图像

### 相机模型

#### 针孔相机模型

- 相机内参矩阵 $\boldsymbol{K}$:

$$
\begin{align}
\left(\begin{array}{l}
u \\
v \\
1
\end{array}\right)=\frac{1}{Z}\left(\begin{array}{ccc}
f_{x} & 0 & c_{x} \\
0 & f_{y} & c_{y} \\
0 & 0 & 1
\end{array}\right)\left(\begin{array}{l}
X \\
Y \\
Z
\end{array}\right) \stackrel{\text { def }}{=} \frac{1}{Z} \boldsymbol{K} \boldsymbol{P}  
\end{align}
$$

- 相机外参矩阵 $\boldsymbol{T}$:
  $$
  Z \boldsymbol{P}_{u v}=Z\left[\begin{array}{c}
  u \\
  v \\
  1
  \end{array}\right]=\boldsymbol{K}\left(\boldsymbol{R} \boldsymbol{P}_{\mathrm{w}}+\boldsymbol{t}\right)=\boldsymbol{K} \boldsymbol{T} \boldsymbol{P}_{\mathrm{w}}
  $$

- 归一化坐标:
  $$
  \left(\boldsymbol{R} \boldsymbol{P}_{\mathrm{w}}+\boldsymbol{t}\right)=\underbrace{[X, Y, Z]^{\mathrm{T}}}_{\text {相机坐标 }} \rightarrow \underbrace{[X / Z, Y / Z, 1]^{\mathrm{T}}}_{\text {归一化坐标 }} .
  $$
  

#### 畸变模型

- 径向畸变

  - 桶形畸变

  - 枕形畸变

  - $$
    \begin{aligned}
    &x_{\text {distorted }}=x\left(1+k_{1} r^{2}+k_{2} r^{4}+k_{3} r^{6}\right) \\
    &y_{\text {distorted }}=y\left(1+k_{1} r^{2}+k_{2} r^{4}+k_{3} r^{6}\right)
    \end{aligned}
    $$

    

- 切向畸变

  - $$
    \begin{aligned}
    x_{\text {distorted }} &=x+2 p_{1} x y+p_{2}\left(r^{2}+2 x^{2}\right) \\
    y_{\text {distorted }} &=y+p_{1}\left(r^{2}+2 y^{2}\right)+2 p_{2} x y
    \end{aligned}
    $$

- 通过5个畸变系数, 将归一化坐标 $[x,y]$ 转换为像素坐标: 

  - $$
    \left\{\begin{array}{l}
    x_{\text {distorted }}=x\left(1+k_{1} r^{2}+k_{2} r^{4}+k_{3} r^{6}\right)+2 p_{1} x y+p_{2}\left(r^{2}+2 x^{2}\right) \\
    y_{\text {distorted }}=y\left(1+k_{1} r^{2}+k_{2} r^{4}+k_{3} r^{6}\right)+p_{1}\left(r^{2}+2 y^{2}\right)+2 p_{2} x y
    \end{array}\right.
    $$

  - $$
    \left\{\begin{array}{l}
    u=f_{x} x_{\text {distorted }}+c_{x} \\
    v=f_{y} y_{\text {distorted }}+c_{y}
    \end{array}\right.
    $$

    

#### 双目相机模型

<img src="VSLAM14Chapter_note.assets/image-20220707221244599.png" alt="image-20220707221244599" style="zoom:50%;" />
$$
\frac{z-f}{z}=\frac{b-u_{\mathrm{L}}+u_{\mathrm{R}}}{b}
$$

$$
z=\frac{f b}{d}, \quad d \stackrel{\text { def }}{=} u_{\mathrm{L}}-u_{\mathrm{R}}
$$

#### RGB-D 相机模型

- 红外结构光 Structured light
  - Kinect Gen1
  - Intel Realsense
- 飞行时间 ToF Time-of-Flight
  - Kinect Gen2
  - ToF cam





## 第六讲 - 非线性优化

### 状态估计问题

#### 批量状态估计与最大后验估计

-   经典SLAM模型: 
    -   $\boldsymbol{x}_{k-1}$, 前一刻相机位姿
    -   $\boldsymbol{u}_k$, 此刻运动控制
    -   $\boldsymbol{w}_k$, 此刻运动噪声
    -   $\boldsymbol{x}_k$, 此刻相机位姿
    -   $\boldsymbol{y}_j$, 此刻观测的路标
    -   $\boldsymbol{v}_{k,j}$, 此刻观测噪声
    -   $\boldsymbol{z}_{k,j}$, 此刻观测结果, 即路标对应到图像上的像素位置 

$$
\left\{\begin{array}{l}
\boldsymbol{x}_{k}=f\left(\boldsymbol{x}_{k-1}, \boldsymbol{u}_{k}\right)+\boldsymbol{w}_{k} \\
\boldsymbol{z}_{k, j}=h\left(\boldsymbol{y}_{j}, \boldsymbol{x}_{k}\right)+\boldsymbol{v}_{k, j}
\end{array}\right.
$$

-   观测方程:

    -   $\boldsymbol{K}$, 相机内参
    -   $s$, 观测路标到图像的距离

    $$
    s \boldsymbol{z}_{k, j}=\boldsymbol{K}\left(\boldsymbol{R}_{k} \boldsymbol{y}_{j}+\boldsymbol{t}_{k}\right)
    $$

-   噪声, 假设满足零均值高斯分布

    -   $\boldsymbol{w}_k$, 运动噪声

    -   $\boldsymbol{v}_{k,j}$, 观测噪声

    -   $\mathcal{N}$, 高斯分布

    -   $0$, 表示零均值

    -   $\boldsymbol{R}_{k}, \boldsymbol{Q}_{k, j}$, 为协方差矩阵
        $$
        \boldsymbol{w}_{k} \sim \mathcal{N}\left(\mathbf{0}, \boldsymbol{R}_{k}\right), \boldsymbol{v}_{k} \sim \mathcal{N}\left(\mathbf{0}, \boldsymbol{Q}_{k, j}\right)
        $$

-   估计方法

    -   增量式
        -   扩展卡尔曼滤波
    -   批量式
        -   局部批量: 滑动窗口法
        -   全局批量: SfM, Structure from Motion



#### 全局批量法SfM

-   考虑从1到N的所有时刻, 假设有M个路标, 定义所有时刻的机器人位姿 $\boldsymbol{x}$ 和路标点坐标 $\boldsymbol{y}$ 为:
    $$
    \boldsymbol{x}=\left\{\boldsymbol{x}_{1}, \ldots, \boldsymbol{x}_{N}\right\}, \quad \boldsymbol{y}=\left\{\boldsymbol{y}_{1}, \ldots, \boldsymbol{y}_{M}\right\}
    $$

-   $\boldsymbol{u}$ 表示所有时刻运动输入, $\boldsymbol{z}$ 表示所有时刻观测数据. 对机器人状态的估计, 从概率学来说, 就是已知输入数据 $\boldsymbol{u}$ 和观测数据 $\boldsymbol{z}$ 的条件下, 求状态 $\boldsymbol{x}$, $\boldsymbol{y}$ 的条件概率分布:
    $$
    P(\boldsymbol{x}, \boldsymbol{y} \mid \boldsymbol{z}, \boldsymbol{u}) .
    $$

    -   当不知道控制输入时
        $$
        P(\boldsymbol{x}, \boldsymbol{y} \mid \boldsymbol{z})
        $$

    -   贝叶斯法则
        $$
        P(\boldsymbol{x}, \boldsymbol{y} \mid \boldsymbol{z}, \boldsymbol{u})=\frac{P(\boldsymbol{z}, \boldsymbol{u} \mid \boldsymbol{x}, \boldsymbol{y}) P(\boldsymbol{x}, \boldsymbol{y})}{P(\boldsymbol{z}, \boldsymbol{u})} \propto \underbrace{P(\boldsymbol{z}, \boldsymbol{u} \mid \boldsymbol{x}, \boldsymbol{y})}_{\text {似然 }} \underbrace{P(\boldsymbol{x}, \boldsymbol{y})}_{\text {先验 }} .
        $$
    
    -   求解最大后验概率, 等价于最大化似然和先验的乘积
        $$
        (\boldsymbol{x}, \boldsymbol{y})^{*}{ }_{\text {MAP }}=\arg \max P(\boldsymbol{x}, \boldsymbol{y} \mid \boldsymbol{z}, \boldsymbol{u})=\arg \max P(\boldsymbol{z}, \boldsymbol{u} \mid \boldsymbol{x}, \boldsymbol{y}) P(\boldsymbol{x}, \boldsymbol{y})
        $$
    
    -   如果没有机器人位姿或路标, 则没有了先验, 此时可以求解最大似然估计 (MLE, Maximize Likelihood Estimation), 即 "在什么样的状态下, 最可能产生现在观测到的数据"
        $$
        (\boldsymbol{x}, \boldsymbol{y})^{*}{ }_{\mathrm{MLE}}=\arg \max P(\boldsymbol{z}, \boldsymbol{u} \mid \boldsymbol{x}, \boldsymbol{y})
        $$



-   最小二乘法求解最大似然估计

    -   观测模型
        $$
        \boldsymbol{z}_{k, j}=h\left(\boldsymbol{y}_{j}, \boldsymbol{x}_{k}\right)+\boldsymbol{v}_{k, j},
        $$

    -   假设噪声项符合高斯分布 $\boldsymbol{v}_{k} \sim \mathcal{N}\left(\mathbf{0}, \boldsymbol{Q}_{k, j}\right)$

    -   观测数据的条件概率依然是高斯分布
        $$
        P\left(\boldsymbol{z}_{j, k} \mid \boldsymbol{x}_{k}, \boldsymbol{y}_{j}\right)=N\left(h\left(\boldsymbol{y}_{j}, \boldsymbol{x}_{k}\right), \boldsymbol{Q}_{k, j}\right)
        $$

    -   高斯分布的概率密度函数
        $$
        P(\boldsymbol{x})=\frac{1}{\sqrt{(2 \pi)^{N} \operatorname{det}(\boldsymbol{\Sigma})}} \exp \left(-\frac{1}{2}(\boldsymbol{x}-\boldsymbol{\mu})^{\mathrm{T}} \boldsymbol{\Sigma}^{-1}(\boldsymbol{x}-\boldsymbol{\mu})\right)
        $$
        
    -   对其取负对数
        $$
        -\ln (P(\boldsymbol{x}))=\frac{1}{2} \ln \left((2 \pi)^{N} \operatorname{det}(\boldsymbol{\Sigma})\right)+\frac{1}{2}(\boldsymbol{x}-\boldsymbol{\mu})^{\mathrm{T}} \boldsymbol{\Sigma}^{-1}(\boldsymbol{x}-\boldsymbol{\mu})
        $$
        
    -   对数函数单调递增, 所以对原函数求最大化, 即对负对数求最小化. 上式中第一项与 $\boldsymbol{x}$ 无关, 只要最小化右侧的二次型项, 就得到状态的最大似然估计. 这里等价于最小化噪声项误差的一个二次型, 为马式距离, 也可以理解为由 $\boldsymbol{Q}_{k, j}^{-1}$ 加权后的欧氏距离.
        $$
        \begin{aligned}
        \left(\boldsymbol{x}_{k}, \boldsymbol{y}_{j}\right)^{*} &=\arg \max \mathcal{N}\left(h\left(\boldsymbol{y}_{j}, \boldsymbol{x}_{k}\right), \boldsymbol{Q}_{k, j}\right) \\
        &=\arg \min \left(\left(\boldsymbol{z}_{k, j}-h\left(\boldsymbol{x}_{k}, \boldsymbol{y}_{j}\right)\right)^{\mathrm{T}} \boldsymbol{Q}_{k, j}^{-1}\left(\boldsymbol{z}_{k, j}-h\left(\boldsymbol{x}_{k}, \boldsymbol{y}_{j}\right)\right)\right)
        \end{aligned}
        $$

    -   假设各个时刻的运动输入和观测互相独立
        $$
        P(\boldsymbol{z}, \boldsymbol{u} \mid \boldsymbol{x}, \boldsymbol{y})=\prod_{k} P\left(\boldsymbol{u}_{k} \mid \boldsymbol{x}_{k-1}, \boldsymbol{x}_{k}\right) \prod_{k, j} P\left(\boldsymbol{z}_{k, j} \mid \boldsymbol{x}_{k}, \boldsymbol{y}_{j}\right)
        $$

    -   定义每次输入和观测数据与模型之间的误差
        $$
        \begin{aligned}
        \boldsymbol{e}_{\boldsymbol{u}, k} &=\boldsymbol{x}_{k}-f\left(\boldsymbol{x}_{k-1}, \boldsymbol{u}_{k}\right) \\
        \boldsymbol{e}_{\boldsymbol{z}, j, k} &=\boldsymbol{z}_{k, j}-h\left(\boldsymbol{x}_{k}, \boldsymbol{y}_{j}\right)
        \end{aligned}
        $$

    -   最小化所有时刻估计值与真实读书之间的马式距离, 等价于求最大似然估计. 这里负对数允许我们把乘积变成求和, 由此变成一个最小二乘问题 (Least Square Problem)
        $$
        \min J(\boldsymbol{x}, \boldsymbol{y})=\sum_{k} \boldsymbol{e}_{\boldsymbol{u}, k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{e}_{\boldsymbol{u}, k}+\sum_{k} \sum_{j} \boldsymbol{e}_{\boldsymbol{z}, k, j}^{\mathrm{T}} \boldsymbol{Q}_{k, j}^{-1} \boldsymbol{e}_{\boldsymbol{z}, k, j}
        $$

    -   SLAM中最小二乘问题的特点

        -   目标函数由许多简单的加权误差项二次型组成
        -   如果用李代数表示增量, 该问题为无约束的最小二乘问题. 如果用旋转矩阵, 则要考虑旋转矩阵自身的约束比如 $\boldsymbol{R}^{\mathrm{T}} \boldsymbol{R}=\boldsymbol{I}$ 且 $\operatorname{det}(\boldsymbol{R})=1$
        -   误差分布将影响每个误差项在整个问题中的权重, 比如某次观测非常准确, 那么该误差项会在问题中占有较高的权重

-   批量状态估计的简单例子: 沿x轴前进或后退的汽车

    -   运动方程和观测方程

        -   $\boldsymbol{x}_k$ 为汽车在k时刻在x轴上的位置
        -   $\boldsymbol{u}_k$ 为运动输入
        -   $\boldsymbol{w}_k$ 为运动噪声
        -   $\boldsymbol{z}_k$ 为对汽车位置在k时刻的测量
        -   $\boldsymbol{n}_k$ 为测量噪声

        $$
        \begin{array}{ll}
        \boldsymbol{x}_{k}=\boldsymbol{x}_{k-1}+\boldsymbol{u}_{k}+\boldsymbol{w}_{k}, & \boldsymbol{w}_{k} \sim \mathcal{N}\left(0, \boldsymbol{Q}_{k}\right) \\
        \boldsymbol{z}_{k}=\boldsymbol{x}_{k}+\boldsymbol{n}_{k}, & \boldsymbol{n}_{k} \sim \mathcal{N}\left(0, \boldsymbol{R}_{k}\right)
        \end{array}
        $$

        -   批量状态变量: $\boldsymbol{x}=\left[\boldsymbol{x}_{0}, \boldsymbol{x}_{1}, \boldsymbol{x}_{2}, \boldsymbol{x}_{3}\right]^{\mathrm{T}}$
        -   批量观测: $\boldsymbol{z}=\left[\boldsymbol{z}_{1}, \boldsymbol{z}_{2}, \boldsymbol{z}_{3}\right]^{\mathrm{T}}$
        -   批量运动输入: $\boldsymbol{u}=\left[\boldsymbol{u}_{1}, \boldsymbol{u}_{2}, \boldsymbol{u}_{3}\right]^{\mathrm{T}}$

    -   最大似然估计为
        $$
        \begin{aligned}
        \boldsymbol{x}_{\text {map }}^{*} &=\arg \max P(\boldsymbol{x} \mid \boldsymbol{u}, \boldsymbol{z})=\arg \max P(\boldsymbol{u}, \boldsymbol{z} \mid \boldsymbol{x}) \\
        &=\prod_{k=1}^{3} P\left(\boldsymbol{u}_{k} \mid \boldsymbol{x}_{k-1}, \boldsymbol{x}_{k}\right) \prod_{k=1}^{3} P\left(\boldsymbol{z}_{k} \mid \boldsymbol{x}_{k}\right)
        \end{aligned}
        $$

        -   运动方程的似然概率
            $$
            P\left(\boldsymbol{u}_{k} \mid \boldsymbol{x}_{k-1}, \boldsymbol{x}_{k}\right)=\mathcal{N}\left(\boldsymbol{x}_{k}-\boldsymbol{x}_{k-1}, \boldsymbol{Q}_{k}\right)
            $$

        -   观测方程的似然概率
            $$
            P\left(\boldsymbol{z}_{k} \mid \boldsymbol{x}_{k}\right)=\mathcal{N}\left(\boldsymbol{x}_{k}, \boldsymbol{R}_{k}\right) .
            $$

        -   误差变量
            $$
            \begin{align}
            \boldsymbol{e}_{\boldsymbol{u}, k} &= \boldsymbol{x}_{k}-\boldsymbol{x}_{k-1}-\boldsymbol{u}_{k}, \\
            
            \boldsymbol{e}_{z, k} &= \boldsymbol{z}_{k}-\boldsymbol{x}_{k}
            \end{align}
            $$

        -   最小二乘的目标函数
            $$
            \min \sum_{k=1}^{3} \boldsymbol{e}_{\boldsymbol{u}, k}^{\mathrm{T}} \boldsymbol{Q}_{k}^{-1} \boldsymbol{e}_{\boldsymbol{u}, k}+\sum_{k=1}^{3} \boldsymbol{e}_{\boldsymbol{z}, k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{e}_{z, k}
            $$

        -   由于该系统是线性的, 很容易写成向量形式, 定义 $\boldsymbol{y}=[\boldsymbol{u}, \boldsymbol{z}]^{\mathrm{T}}$, 写出矩阵 $\boldsymbol{H}$, 使得:
            $$
            \begin{align}
            \boldsymbol{y}-\boldsymbol{H} \boldsymbol{x} &= 
            \boldsymbol{e} \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma})
            
            \\
            \\
            
            \boldsymbol{H} &= \left[\begin{array}{cccc}
            1 & -1 & 0 & 0 \\
            0 & 1 & -1 & 0 \\
            0 & 0 & 1 & -1 \\
            0 & 1 & 0 & 0 \\
            0 & 0 & 1 & 0 \\
            0 & 0 & 0 & 1
            \end{array}\right],
            
            \\
            \\
            
            \boldsymbol{\Sigma} &= \operatorname{diag}\left(\boldsymbol{Q}_{1}, \boldsymbol{Q}_{2}, \boldsymbol{Q}_{3}, \boldsymbol{R}_{1}, \boldsymbol{R}_{2}, \boldsymbol{R}_{3}\right)
            
            
            \end{align}
            $$

        -   整个问题可以写成
            $$
            \boldsymbol{x}_{\text {map }}^{*}=\arg \min \boldsymbol{e}^{\mathrm{T}} \boldsymbol{\Sigma}^{-1} \boldsymbol{e}
            
            \\
            $$

        -   唯一解
            $$
            \boldsymbol{x}_{\text {map }}^{*}=\left(\boldsymbol{H}^{\mathrm{T}} \boldsymbol{\Sigma}^{-1} \boldsymbol{H}\right)^{-1} \boldsymbol{H}^{\mathrm{T}} \boldsymbol{\Sigma}^{-1} \boldsymbol{y} .
            $$





### 非线性最小二乘

#### 简单的最小二乘问题


$$
\min _{\boldsymbol{x}} F(\boldsymbol{x})=\frac{1}{2}\|f(\boldsymbol{x})\|_{2}^{2}
$$
其中, 自变量 $\boldsymbol{x} \in \mathbb{R}^{n}, f$ 是任意标量非线性函数 $f(\boldsymbol{x}): \mathbb{R}^{n} \mapsto \mathbb{R}$ 。

这样一个优化问题。显然, 如果 $f$ 是个数学形式上很简单的函数, 那么该问题可以用解析形式来 求。令目标函数的导数为零, 然后求解 $\boldsymbol{x}$ 的最优值, 就和求二元函数的极值一样:
$$
\frac{\mathrm{d} F}{\mathrm{~d} \boldsymbol{x}}=\mathbf{0} .
$$
如果 $f$ 为简单的线形函数, 那么这个问题就是简单的线形最小二乘问题, 但有些导函数形式复杂, 可以采用迭代法:

![image-20220708174107700](VSLAM14Chapter_note.assets/image-20220708174107700.png)



#### 一阶和二阶梯度法

现在考虑第 $k$ 次迭代, 假设我们在 $\boldsymbol{x}_{k}$ 处, 想要寻到增量 $\Delta \boldsymbol{x}_{k}$, 那么最直观的方式是将目标函数在 $x_{k}$ 附近进行泰勒展开:
$$
F\left(\boldsymbol{x}_{k}+\Delta \boldsymbol{x}_{k}\right) \approx F\left(\boldsymbol{x}_{k}\right)+\boldsymbol{J}\left(\boldsymbol{x}_{k}\right)^{\mathrm{T}} \Delta \boldsymbol{x}_{k}+\frac{1}{2} \Delta \boldsymbol{x}_{k}^{\mathrm{T}} \boldsymbol{H}\left(\boldsymbol{x}_{k}\right) \Delta \boldsymbol{x}_{k} .
$$
其中 $\boldsymbol{J}\left(\boldsymbol{x}_{k}\right)$ 是 $F(\boldsymbol{x})$ 关于 $\boldsymbol{x}$ 的一阶导数 (也叫梯度、雅可比（Jacobian ) 矩阵), $ \boldsymbol{H}$ 则是二阶导数 (海塞 (Hessian) 矩阵), 它们都在 $\boldsymbol{x}_{k}$ 处取值.

-   一阶梯度下降
    $$
    \Delta \boldsymbol{x}^{*}=-\boldsymbol{J}\left(\boldsymbol{x}_{k}\right)
    $$

-   二阶梯度下降
    $$
    \Delta \boldsymbol{x}^{*}=\arg \min \left(F(\boldsymbol{x})+\boldsymbol{J}(\boldsymbol{x})^{\mathrm{T}} \Delta \boldsymbol{x}+\frac{1}{2} \Delta \boldsymbol{x}^{\mathrm{T}} \boldsymbol{H} \Delta \boldsymbol{x}\right)
    $$
    求右侧等式关于 $\Delta \boldsymbol{x}$ 的导数并令它为零, 求解这个线性方程, 就得到了增量。该方法又称为牛顿法。
    $$
    \boldsymbol{J}+\boldsymbol{H} \Delta \boldsymbol{x}=\mathbf{0} \Rightarrow \boldsymbol{H} \Delta \boldsymbol{x}=-\boldsymbol{J}
    $$



-   一阶和二阶梯度法的特点
    -   把函数在迭代点附近进行泰勒展开, 针对更新量做最小化
    -   一阶梯度最速下降过于贪心, 容易走出锯齿, 增加迭代次数
    -   二阶梯度牛顿法需要计算海森矩阵, 计算消耗大



#### 高斯牛顿法

-   将 $f(\boldsymbol{x})$ 进行一阶泰勒展开

$$
f(\boldsymbol{x}+\Delta \boldsymbol{x}) \approx f(\boldsymbol{x})+\boldsymbol{J}(\boldsymbol{x})^{\mathrm{T}} \Delta \boldsymbol{x}
$$

-   这里 $\boldsymbol{J}(\boldsymbol{x})^{\mathrm{T}}$ 为 $f(\boldsymbol{x})$ 关于 $\boldsymbol{x}$ 的导数, 为 $n \times 1$ 的列向量。根据前面的框架, 当前的目标是寻找增量 $\Delta \boldsymbol{x}$, 使得 $\|f(\boldsymbol{x}+\Delta \boldsymbol{x})\|^{2}$ 达到最小。为了求 $\Delta \boldsymbol{x}$, 我们需要解一个线性的最小二乘问题:
    $$
    \Delta \boldsymbol{x}^{*}=\arg \min _{\Delta \boldsymbol{x}} \frac{1}{2}\left\|f(\boldsymbol{x})+\boldsymbol{J}(\boldsymbol{x})^{\mathrm{T}} \Delta \boldsymbol{x}\right\|^{2}
    $$

-   根据极值条件, 将上述目标函数对 $ \Delta \boldsymbol{x}$ 求导, 并令导数为零, 为此先展开目标函数平方项:
    $$
    \begin{aligned}
    \frac{1}{2}\left\|f(\boldsymbol{x})+\boldsymbol{J}(\boldsymbol{x})^{\mathrm{T}} \Delta \boldsymbol{x}\right\|^{2} &=\frac{1}{2}\left(f(\boldsymbol{x})+\boldsymbol{J}(\boldsymbol{x})^{\mathrm{T}} \Delta \boldsymbol{x}\right)^{\mathrm{T}}\left(f(\boldsymbol{x})+\boldsymbol{J}(\boldsymbol{x})^{\mathrm{T}} \Delta \boldsymbol{x}\right) \\
    &=\frac{1}{2}\left(\|f(\boldsymbol{x})\|_{2}^{2}+2 f(\boldsymbol{x}) \boldsymbol{J}(\boldsymbol{x})^{\mathrm{T}} \Delta \boldsymbol{x}+\Delta \boldsymbol{x}^{\mathrm{T}} \boldsymbol{J}(\boldsymbol{x}) \boldsymbol{J}(\boldsymbol{x})^{\mathrm{T}} \Delta \boldsymbol{x}\right) .
    \end{aligned}
    $$

-   求上式关于 $\Delta \boldsymbol{x}$ 的导数, 并令其为零:

$$
\boldsymbol{J}(\boldsymbol{x}) f(\boldsymbol{x})+\boldsymbol{J}(\boldsymbol{x}) \boldsymbol{J}^{\mathrm{T}}(\boldsymbol{x}) \Delta \boldsymbol{x}=\mathbf{0}
$$

$$
\underbrace{\boldsymbol{J}(\boldsymbol{x}) \boldsymbol{J}^{\mathrm{T}}}_{\boldsymbol{H}(\boldsymbol{x})}(\boldsymbol{x}) \Delta \boldsymbol{x}=\underbrace{-\boldsymbol{J}(\boldsymbol{x}) f(\boldsymbol{x})}_{\boldsymbol{g}(\boldsymbol{x})} .
$$

-   这个方程是关于变量 $\Delta \boldsymbol{x}$ 的线性方程组, 我们称它为增量方程, 也可以称为高斯牛顿方程 ( Gauss-Newton equation) 或者正规方程 (Normal equation)。我们把左边的系数定义为 $\boldsymbol{H}$, 右边定义为 $\boldsymbol{g}$, 那么上式变为:
    $$
    \boldsymbol{H} \Delta \boldsymbol{x}=\boldsymbol{g}
    $$

-   这里把左侧记作 $\boldsymbol{H}$ 是有意义的。对比牛顿法可见, 高斯牛顿法用 $\boldsymbol{J} \boldsymbol{J}^{\mathrm{T}}$ 作为牛顿法中二阶 Hessian 矩阵的近似, 从而省略了计算 $\boldsymbol{H}$ 的过程。求解增量方程是整个优化问题的核心所在.

-   高斯牛顿步骤

    1. 给定初始值 $\boldsymbol{x}_{0}$ 。
    2. 对于第 $k$ 次迭代, 求出当前的雅可比矩阵 $\boldsymbol{J}\left(\boldsymbol{x}_{k}\right)$ 和误差 $f\left(\boldsymbol{x}_{k}\right)$ 。
    3. 求解增量方程: $\boldsymbol{H} \Delta \boldsymbol{x}_{k}=\boldsymbol{g}$ 。
    4. 若 $\Delta \boldsymbol{x}_{k}$ 足够小, 则停止。否则, 令 $\boldsymbol{x}_{k+1}=\boldsymbol{x}_{k}+\Delta \boldsymbol{x}_{k}$, 返回第 2 步。

-   高斯牛顿缺点

    -   $\boldsymbol{J} \boldsymbol{J}^{\mathrm{T}}$ 只有半正定性, 可能为奇异矩阵或病态矩阵, 稳定性较差, 导致算法不收敛
    -   原函数在这个点的局部近似不像二次函数
    -   如果 $\Delta \boldsymbol{x}$ 步长太大, 也会导致局部近似不够准确.
    -   一些线搜索法加入了步长 $\alpha$ 
    -   列文伯格一马夸尔特方法在一定程度上修正了这些问题, 一般认为它比高斯牛顿法更加健壮, 但它的收玫速度可能比高斯牛顿法更慢, 被称为阻尼牛顿法



#### 列文伯格一马夸尔特方法

##### 信赖区域

-   相比高斯牛顿, 给 $\Delta\boldsymbol{x}$ 添加一个范围, 信赖区域 (Trust Region)

-   那么, 如何确定这个信赖区域的范围呢? 一个比较好的方法是根据我们的近似模型跟实际函数之间的差异来确定: 如果差异小, 说明近似效果好, 我们扩大近似的范围; 反之, 如果差异大, 就缩小近似的范围。我们定义一个指标 $\rho$ 来刻画近似的好坏程度
    $$
    \rho=\frac{f(\boldsymbol{x}+\Delta \boldsymbol{x})-f(\boldsymbol{x})}{\boldsymbol{J}(\boldsymbol{x})^{\mathrm{T}} \Delta \boldsymbol{x}}
    $$

-   $\rho$ 的分子是实际函数下降的值, 分母是近似模型下降的值。如果 $\rho$ 接近于 1, 则近似是好的。如 果 $\rho$ 太小, 说明实际减小的值远少于近似减小的值, 则认为近似比较差, 需要缩小近似范围。反 之, 如果 $\rho$ 比较大, 则说明实际下降的比预计的更大，我们可以放大近似范围。

##### 算法步骤

1. 给定初始值 $x_{0}$, 以及初始优化半径 $\mu_{\text {。 }}$

2. 对于第 $k$ 次迭代, 在高斯牛顿法的基础上加上信赖区域, 求解:
    $$
    \min _{\Delta \boldsymbol{x}_{k}} \frac{1}{2}\left\|f\left(\boldsymbol{x}_{k}\right)+\boldsymbol{J}\left(\boldsymbol{x}_{k}\right)^{\mathrm{T}} \Delta \boldsymbol{x}_{k}\right\|^{2}, \quad \text { s.t. } \quad\left\|\boldsymbol{D} \Delta \boldsymbol{x}_{k}\right\|^{2} \leqslant \mu,
    $$
    其中, $\mu$ 是信赖区域的半径, $\boldsymbol{D}$ 为系数矩阵. 可以理解为把增量限定于一个半径为 $\mu$ 的球中, 带上 $\boldsymbol{D}$ 之后可以看成椭球, 比如可以为 $\boldsymbol{I}$ 即为球. 这里通过拉格朗日乘子 $\lambda$ 来求解带不等式约束的梯度优化问题。
    $$
    \mathcal{L}\left(\Delta \boldsymbol{x}_{k}, \lambda\right) =\frac{1}{2}\left\|f\left(\boldsymbol{x}_{k}\right) + \boldsymbol{J}\left(\boldsymbol{x}_{k}\right)^{\mathrm{T}} \Delta \boldsymbol{x}_{k}\right\|^{2}+\frac{\lambda}{2}\left(\left\|\boldsymbol{D} \Delta \boldsymbol{x}_{k}\right\|^{2}-\mu\right)
    $$
    
    $$
    \begin{align}
    (\boldsymbol{H} + \lambda \boldsymbol{D}^{\mathrm{T}}\boldsymbol{D})\Delta \boldsymbol{x}_k &= \boldsymbol{g} 
    
    \\
    \\
    
    (\boldsymbol{H} + \lambda \boldsymbol{I})\Delta \boldsymbol{x}_k &= \boldsymbol{g}, \text{with }\boldsymbol{D} = \boldsymbol{I} \\
    
    
    
    \end{align}
    $$
    我们看到, 一方面, 当参数 $\lambda$ 比较小时, $H$ 占主要地位, 这说明二次近似模型在该范围内 是比较好的, 列文伯格一马夸尔特方法更接近于高斯牛顿法。另一方面, 当 $\lambda$ 比较大时, $\lambda \boldsymbol{I}$ 占 据主要地位, 列文伯格一马夸尔特方法更接近于一阶梯度下降法（即最速下降）, 这说明附近的 二次近似不够好。列文伯格一马㚎尔特方法的求解方式, 可在一定程度上避免线性方程组的系数 矩阵的非奇异和病态问题, 提供更稳定、更准确的增量 $\Delta \boldsymbol{x}_{\text {。 }}$

3. 计算 $\rho_{\circ}$
    $$
    \rho=\frac{f(\boldsymbol{x}+\Delta \boldsymbol{x})-f(\boldsymbol{x})}{\boldsymbol{J}(\boldsymbol{x})^{\mathrm{T}} \Delta \boldsymbol{x}}
    $$

4. 若 $\rho>\frac{3}{4}$, 则设置 $\mu=2 \mu_{\circ}$

5. 若 $\rho<\frac{1}{4}$, 则设置 $\mu=0.5 \mu_{\text {。 }}$

6. 如果 $\rho$ 大于某阈值, 则认为近似可行。令 $\boldsymbol{x}_{k+1}=\boldsymbol{x}_{k}+\Delta \boldsymbol{x}_{k}$ 。

7. 判断算法是否收敛。如不收玫则返回第 2 步, 否则结束。





### 实践 - 曲线拟合问题

#### 手写高斯牛顿法

-   待拟合的曲线模型, $a, b, c$ 为曲线的参数, $w$ 为高斯噪声, 满足 $w \sim\left(0, \sigma^{2}\right)$ 。
    $$
    y=\exp \left(a x^{2}+b x+c\right)+w
    $$

-   目标函数
    $$
    \min _{a, b, c} \frac{1}{2} \sum_{i=1}^{N}\left\|y_{i}-\exp \left(a x_{i}^{2}+b x_{i}+c\right)\right\|^{2}
    $$

-   误差项
    $$
    e_{i}=y_{i}-\exp \left(a x_{i}^{2}+b x_{i}+c\right)
    $$

-   雅可比矩阵
    $$
    \begin{align}
    \boldsymbol{J}_{i}&=\left[\frac{\partial e_{i}}{\partial a}, \frac{\partial e_{i}}{\partial b}, \frac{\partial e_{i}}{\partial c}\right]^{\mathrm{T}}
    
    \\
    \\
    
    \frac{\partial e_{i}}{\partial a}&=-x_{i}^{2} \exp \left(a x_{i}^{2}+b x_{i}+c\right) \\
    \frac{\partial e_{i}}{\partial b}&=-x_{i} \exp \left(a x_{i}^{2}+b x_{i}+c\right) \\
    \frac{\partial e_{i}}{\partial c}&=-\exp \left(a x_{i}^{2}+b x_{i}+c\right)
    
    \end{align}
    $$

-   高斯牛顿法的增量方程为
    $$
    \left(\sum_{i=1}^{100} \boldsymbol{J}_{i}\left(\sigma^{2}\right)^{-1} \boldsymbol{J}_{i}^{\mathrm{T}}\right) \Delta \boldsymbol{x}_{k}=\sum_{i=1}^{100}-\boldsymbol{J}_{i}\left(\sigma^{2}\right)^{-1} e_{i}
    $$

-   求解 $\boldsymbol{A}\boldsymbol{x} = \boldsymbol{b}$
    -   LU分解, 求解 $\boldsymbol{L}\boldsymbol{U}\boldsymbol{x} = \boldsymbol{b}$
    -   LUP分解, 求解 $\boldsymbol{L}\boldsymbol{U}\boldsymbol{P}\boldsymbol{x} = \boldsymbol{b}$
    -   Cholesky分解, 当A是SPD 对称正定矩阵时, 求解 $\boldsymbol{L}\boldsymbol{L}^{\text{T}}\boldsymbol{x} = \boldsymbol{b}$ 



#### Ceres曲线拟合, 最小二乘求解器

-   Ceres求解的最小二乘问题, 带边界的核函数最小二乘
    $$
    \begin{aligned}
    &\min _{x} \frac{1}{2} \sum_{i} \rho_{i}\left(\left\|f_{i}\left(x_{i_{1}}, \cdots, x_{i_{n}}\right)\right\|^{2}\right) \\
    &\text { s.t. } l_{j} \leqslant x_{j} \leqslant u_{j} .
    \end{aligned}
    $$

    -   参数块, 即优化变量: $x_{1}, \cdots, x_{n}$
    -   残差块, 即代价函数或误差项: $f_i$ 
    -   优化变量的上限或下限: $l_j$, $u_j$
    -   核函数: $\boldsymbol{\rho}(\cdot)$

-   Ceres求解步骤

    -   定义参数块, 可以是向量, 四元数, 李代数等
    -   定义残差块
    -   定义残差块的雅可比计算方式
    -   设置Problem对象, 设置配置信息如迭代次数, 中止条件等
    -   调用Solve求解





#### G2O曲线拟合, 基于图优化的非线性优化

##### 图优化理论简介

-   图优化: 把优化问题表现成图, 能直观看到优化问题的表现
-   图, 贝叶斯图, 因子图: 
    -   顶点Vertex, 优化变量
    -   边Edge, 误差项

![image-20220713160400608](VSLAM14Chapter_note.assets/image-20220713160400608.png)



##### 使用G2O拟合曲线

-   当前曲线拟合问题对应的因子图, 只有一个顶点, 带优化的参数 $a,b,c$
    $$
    y=\exp \left(a x^{2}+b x+c\right)+w
    $$
    

![image-20220713161309959](VSLAM14Chapter_note.assets/image-20220713161309959.png)

-   G2O 优化步骤
    1.  定义顶点和边的类型
    2.  构建图
    3.  选择优化算法
    4.  调用G2O优化









## 第七讲 - 视觉里程计 1

### 特征点法 (以ORB特征为例)

#### 特征点

-   特质
    -   可重复性
    -   可区别性
    -   高效率
    -   本地性
-   特征点组成
    -   关键点 Key-Point, 特征点在图像中的位置
    -   描述子 Descriptor, 描述了关键点周围像素的信息
        -   外观相思的特征应该有相似的描述子
-   经典的特征点
    -   SIFT, (Scale-Invariant Feature Transform)
        -   尺度不变特征变换
        -   计算量大
    -   SURF
    -   FAST 关键点
        -   没有描述子
    -   ORB, (Oriented FAST and Rotated BRIEF)
        -   改进了FAST检测子没有方向性的问题
        -   速度极快的二进制描述子BRIEF (Binary Robust Independent Elementary Feature)

#### ORB特征

ORB特征的组成

-   关键点 - Oriented FAST

    -   改进的FAST角点, 提供了特征点的主方向

    -   只比较像素与领域的亮度大小

    -   FAST 提取步骤

        1.  在图像中选取像素 $p$, 假设它的亮度为 $I_{p}$ 。
        2.  设置一个阈值 $T$ ( 比如, $I_{p}$ 的 $20 \%$ )。
        3.  以像素 $p$ 为中心，选取半径为 3 的圆上的 16 个像素点。
        4.  假如选取的圆上有连续的 $N$ 个点的亮度大于 $I_{p}+T$ 或小于 $I_{p}-T$, 那么像素 $p$ 可以被 认为是特征点 ( $N$ 通常取 12, 即 FAST-12。其他常用的 $N$ 取值为 9 和 11 , 它们分别被 称为 FAST-9 和 FAST-11)。
            1.  FAST-12 预测试操作, 直接检测1, 5, 9, 13的像素亮度, 只有当这四个中有三个同时大于 $I_{p}+T$ 或小于 $I_{p}-T$, 才有可能为角点, 大大加速了角点检测
            2.  非极大值抑制, 避免角点集中
        5.  循环以上四步, 对每一个像素执行相同的操作。

    -   ![image-20220713181633028](VSLAM14Chapter_note.assets/image-20220713181633028.png)

    -   针对FAST角点不具有方向性和尺度的问题, ORB添加了尺度和旋转的描述, 称为Orient FAST

        -   尺度: 在图像金字塔每一层上检测角点, 匹配不同层的特征来实现尺度变化比如前进或后退时的特征匹配

        -   旋转: 灰度质心法 Intensity Centroid, 计算特征点附近的图像灰度质心

            1.  在一个小的图像块B中, 定义图像块的矩为
                $$
                m_{pq} = \sum_{x,y\in B} x^p y^q I(x,y), \quad p,q={0,1}
                $$

            2.  通过矩可以找到图像块的质心:
                $$
                C=\left(\frac{m_{10}}{m_{00}}, \frac{m_{01}}{m_{00}}\right) .
                $$

            3.  连接图像块的几何中心 $O$ 与质心 $C$, 得到一个方向向量 $\overrightarrow{O C}$, 于是特征点的方向可以 定义为
                $$
                \theta=\arctan \left(m_{01} / m_{10}\right) .
                $$

-   描述子 - BRIEF (Binary Robust Independent Elementary Feature)

    -   BRIEF 是一种二进制描述子, 其描述向量由许多个 0 和 1 组成, 这里的 0 和 1 编码了关键点 附近两个随机像素 (比如 $p$ 和 $q$ ) 的大小关系: 如果 $p$ 比 $q$ 大, 则取 1 , 反之就取 0 。如果我们取 了 128 个这样的 $p, q$, 则最后得到 128 维由 $0 、 1$ 组成的向量 ${ }^{[44]}$ 。BRIEF 使用了随机选点的比较, 速度非常快, 而且由于使用了二进制表达, 存储起来也十分方便, 适用于实时的图像匹配。原始 的 BRIEF 描述子不具有旋转不变性, 因此在图像发生旋转时容易丢失。而 ORB 在 FAST 特征点 提取阶段计算了关键点的方向, 所以可以利用方向信息, 计算旋转之后的 “Steer BRIEF” 特征使 ORB 的描述子具有较好的旋转不变性。



#### 特征匹配

-   匹配方式
    -   暴力匹配
    -   快速近似最近邻 (FLANN)
-   匹配距离
    -   欧式距离
    -   汉明距离, 两个二进制串之间的不同位数的个数



#### 计算机相机运动

根据相机原理和获取的不同匹配点, 分为以下情况来估计相机的运动

-   2D-2D: 单目相机, 两帧图像获取两组2D点,估计运动, 使用**对极几何**
-   3D-3D: 双目/RGB-D/ToF 等深度相机, 两组3D点估计运动, 使用**ICP**
-   3D-2D: 3D投影到2D, 估计相机运动, 使用**PnP**



### 2D-2D 对极几何

#### 对极约束

-   两帧图像$I_1$, $I_2$
-   第一帧到第二帧的相机运动 $\boldsymbol{R}$, $\boldsymbol{t}$
-   相机中心点 $O_1$, $O_2$
-   空间中 $P$ 点在 $I_1$ 中的投影点 $p_1$, 对应 $I_2$ 中的投影点 $p_2$, 两者通过特征匹配
-   极平面 (Epipolar plane) $O_1O_2P$
-   极点 (Epipoles) $e_1$, $e_2$
-   极线 (Epipolar line) $l_1$, $l_2$

-   基线 $O_1O_2$

![image-20220714161605498](VSLAM14Chapter_note.assets/image-20220714161605498.png)

-   几何关系

    -    $P$ 的空间位置
         $$
         \boldsymbol{P} = [X,Y,Z]^{\text{T}}
         $$

    -    $\boldsymbol{p}_1$, $\boldsymbol{p}_2$ 像素位置

         -   $\boldsymbol{K}$ 相机内参矩阵
         -   $\boldsymbol{R}$, $\boldsymbol{t}$ 相机运动  

         $$
         s_{1} \boldsymbol{p}_{1}=\boldsymbol{K} \boldsymbol{P}, \quad s_{2} \boldsymbol{p}_{2}=\boldsymbol{K}(\boldsymbol{R} \boldsymbol{P}+\boldsymbol{t})
         $$

    -    尺度意义下相等的齐次坐标, 得到归一化平面上的坐标 $\boldsymbol{x}_1$, $\boldsymbol{x}_2$
         $$
         s \boldsymbol{p} \simeq \boldsymbol{p} 
         
         \\
         
         \boldsymbol{p}_{1} \simeq \boldsymbol{K} \boldsymbol{P}, \quad \boldsymbol{p}_{2} \simeq \boldsymbol{K}(\boldsymbol{R P}+\boldsymbol{t})
         
         \\
         
         \boldsymbol{x}_{1}=\boldsymbol{K}^{-1} \boldsymbol{p}_{1}, \quad \boldsymbol{x}_{2}=\boldsymbol{K}^{-1} \boldsymbol{p}_{2}
         
         \\
         
         \boldsymbol{x}_2 \simeq \boldsymbol{R} \boldsymbol{x}_1 + \boldsymbol{t}
         
         \\
         
         \boldsymbol{t}^{\wedge}\boldsymbol{x}_2 \simeq \boldsymbol{t}^{\wedge}\boldsymbol{R}\boldsymbol{x}_1
         
         \\
         
         \boldsymbol{x}_2^{\text{T}}\boldsymbol{t}^{\wedge}\boldsymbol{x}_2 \simeq \boldsymbol{x}_2^{\text{T}}\boldsymbol{t}^{\wedge}\boldsymbol{R}\boldsymbol{x}_1
         $$

    -    对极约束方程
         $$
         \boldsymbol{x}_2^{\text{T}}\boldsymbol{t}^{\wedge}\boldsymbol{R}\boldsymbol{x}_1 = 0
         
         \\
         
         \boldsymbol{p}_2^{\wedge}\boldsymbol{K}^{-\text{T}}\boldsymbol{t}^{\wedge}\boldsymbol{R}\boldsymbol{K}^{-1}\boldsymbol{p}_1 = 0
         $$

    -    基础矩阵 $\boldsymbol{F}$ 和本质矩阵 $\boldsymbol{E}$

         -   本质矩阵 $\boldsymbol{E} = \boldsymbol{t}^{\wedge}\boldsymbol{R}$ 
         -   基础矩阵 $\boldsymbol{F} = \boldsymbol{K}^{-\text{T}}\boldsymbol{E}\boldsymbol{K}^{-1}$ 
         -   对极约束 $\boldsymbol{x}_2^{\text{T}}\boldsymbol{E}\boldsymbol{x}_1 = \boldsymbol{p}_2^{\text{T}}\boldsymbol{F}\boldsymbol{p}_1 = 0$

    -    相机位置估计问题步骤

         1.  根据配对点像素位置求出 $\boldsymbol{E}$ 或 $\boldsymbol{F}$
             1.  八点法
         2.  根据 $\boldsymbol{E}$ 或 $\boldsymbol{F}$ 求出 $\boldsymbol{R}$ , $\boldsymbol{t}$
             1.  SVD求解




#### 本质矩阵 Essential Matrix

-   本质矩阵 $\boldsymbol{E} = \boldsymbol{t}^{\wedge}\boldsymbol{R} \in \R^{3x3}$

-   本质矩阵性质
    -   $\boldsymbol{E}$ 在不同尺度下等价, 即乘以任意非零常数, 对极约束仍然满足
    -   $\boldsymbol{E}$ 的奇异值形式为 $[\sigma, \sigma, 0]^{\text{T}}$
    -   $\boldsymbol{t}^{\wedge}\boldsymbol{R}$ 平移加旋转6个自由度, 但考虑尺度等价性, 实际是5个自由度, 表明最少五个点可以求解 $\boldsymbol{E}$, 但一般用**八点法**来求解. 

-   八点法求解本质矩阵 $\boldsymbol{E}$

    -   考虑一对匹配点, 归一化坐标为 $\boldsymbol{x}_{1}=\left[u_{1}, v_{1}, 1\right]^{\mathrm{T}}, \boldsymbol{x}_{2}=\left[u_{2}, v_{2}, 1\right]^{\mathrm{T}}$ 

    -   根据对极约束
        $$
        \left(u_{2}, v_{2}, 1\right)\left(\begin{array}{ccc}
        e_{1} & e_{2} & e_{3} \\
        e_{4} & e_{5} & e_{6} \\
        e_{7} & e_{8} & e_{9}
        \end{array}\right)\left(\begin{array}{c}
        u_{1} \\
        v_{1} \\
        1
        \end{array}\right)=0
        $$

    -   把矩阵 $\boldsymbol{E}$ 展开为向量形式
        $$
        \boldsymbol{e}=\left[e_{1}, e_{2}, e_{3}, e_{4}, e_{5}, e_{6}, e_{7}, e_{8}, e_{9}\right]^{\mathrm{T}}
        $$

    -   对极约束写为关于 $\boldsymbol{e}$ 的线性形式
        $$
        \left[u_{2} u_{1}, u_{2} v_{1}, u_{2}, v_{2} u_{1}, v_{2} v_{1}, v_{2}, u_{1}, v_{1}, 1\right] \cdot \boldsymbol{e}=0
        $$

    -   加入其他点
        $$
        \left(\begin{array}{ccccccccc}
        u_{2}^{1} u_{1}^{1} & u_{2}^{1} v_{1}^{1} & u_{2}^{1} & v_{2}^{1} u_{1}^{1} & v_{2}^{1} v_{1}^{1} & v_{2}^{1} & u_{1}^{1} & v_{1}^{1} & 1 \\
        u_{2}^{2} u_{1}^{2} & u_{2}^{2} v_{1}^{2} & u_{2}^{2} & v_{2}^{2} u_{1}^{2} & v_{2}^{2} v_{1}^{2} & v_{2}^{2} & u_{1}^{2} & v_{1}^{2} & 1 \\
        \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \\
        u_{2}^{8} u_{1}^{8} & u_{2}^{8} v_{1}^{8} & u_{2}^{8} & v_{2}^{8} u_{1}^{8} & v_{2}^{8} v_{1}^{8} & v_{2}^{8} & u_{1}^{8} & v_{1}^{8} & 1
        \end{array}\right)\left(\begin{array}{l}
        e_{1} \\
        e_{2} \\
        e_{3} \\
        e_{4} \\
        e_{5} \\
        e_{6} \\
        e_{7} \\
        e_{8} \\
        e_{9}
        \end{array}\right)=0
        $$

    -   8对匹配点组成的矩阵满足秩为8时, 能解得 $\boldsymbol{e}$ , 通过奇异值分解恢复出 $\boldsymbol{R}, \boldsymbol{t}$
        $$
        \boldsymbol{E}=\boldsymbol{U} \boldsymbol{\Sigma} \boldsymbol{V}^{\mathrm{T}}
        $$

    -   其中 $\boldsymbol{U}, \boldsymbol{V}$ 为正交阵, $\boldsymbol{\Sigma}$ 为奇异值矩阵。根据 $\boldsymbol{E}$ 的内在性质, 我们知道 $\boldsymbol{\Sigma}=\operatorname{diag}(\sigma, \sigma, 0)$ 。在 SVD 分解中, 对于任意一个 $E$, 存在四个可能的 $t, R$ 与它对应
        $$
        \begin{aligned}
        \boldsymbol{t}_{1}^{\wedge} &=\boldsymbol{U} \boldsymbol{R}_{Z}\left(\frac{\pi}{2}\right) \boldsymbol{\Sigma} \boldsymbol{U}^{\mathrm{T}}, \quad \boldsymbol{R}_{1}=\boldsymbol{U} \boldsymbol{R}_{Z}^{\mathrm{T}}\left(\frac{\pi}{2}\right) \boldsymbol{V}^{\mathrm{T}} 
        
        \\
        
        \boldsymbol{t}_{2}^{\wedge} &=\boldsymbol{U} \boldsymbol{R}_{Z}\left(-\frac{\pi}{2}\right) \boldsymbol{\Sigma} \boldsymbol{U}^{\mathrm{T}}, \quad \boldsymbol{R}_{2}=\boldsymbol{U} \boldsymbol{R}_{Z}^{\mathrm{T}}\left(-\frac{\pi}{2}\right) \boldsymbol{V}^{\mathrm{T}}
        
        \\
        
        \boldsymbol{t}_3^{\wedge} &= -\boldsymbol{U} \boldsymbol{R}_{Z}\left(\frac{\pi}{2}\right) \boldsymbol{\Sigma} \boldsymbol{U}^{\mathrm{T}}
        
        \\ 
        
        \boldsymbol{t}_4^{\wedge} &= -\boldsymbol{U} \boldsymbol{R}_{Z}\left(-\frac{\pi}{2}\right) \boldsymbol{\Sigma} \boldsymbol{U}^{\mathrm{T}}
        
        \end{aligned}
        $$
        ![image-20220718093345417](VSLAM14Chapter_note.assets/image-20220718093345417.png)

    -   只有第一种解中, $P$ 点的深度值在两个相机中都为正, 所以只有一个解

    -   如果根据线性方程解出的 $\boldsymbol{E}$ 不满足其应有的内在性质, 即如果 $\boldsymbol{E}$ 的奇异值不为 $\sigma, \sigma, 0$ 的形式, 那么则对其奇异值矩阵 $\text{diag}(\sigma_1, \sigma_2, \sigma_3), \sigma_1 \geq \sigma_2 \geq \sigma_3$ 做如下处理
        $$
        \boldsymbol{E}=\boldsymbol{U} \operatorname{diag}\left(\frac{\sigma_{1}+\sigma_{2}}{2}, \frac{\sigma_{1}+\sigma_{2}}{2}, 0\right) \boldsymbol{V}^{\mathrm{T}}
        $$

    -   由于 $\boldsymbol{E}$ 具有尺度不变性, 所以直接设置为 $\text{diag}(1,1,0)$ 同样可行

        



#### 单应矩阵 Homography Matrix

-   单应矩阵描述了两个平面之间的映射关系. 如果场景中的特征点在同一平面上, 则可以通过单应性进行运动估计

-   假设图像 $\boldsymbol{I}_1$, $\boldsymbol{I}_2$ 有一对匹配好的特征点 $p_1, p_2$, 对应平面 $P$ 上, 该平面满足方程
    $$
    \boldsymbol{n}^{\text{T}}\boldsymbol{P} + d = 0
    
    \\
    
    \frac{\boldsymbol{n}^{\text{T}}\boldsymbol{P}}{d} = 1
    $$

-   代入 $\boldsymbol{p}_2$
    $$
    \begin{aligned}
    \boldsymbol{p}_{2} & \simeq \boldsymbol{K}(\boldsymbol{R P}+\boldsymbol{t}) \\
    & \simeq \boldsymbol{K}\left(\boldsymbol{R P}+\boldsymbol{t} \cdot\left(-\frac{\boldsymbol{n}^{\mathrm{T}} \boldsymbol{P}}{d}\right)\right) \\
    & \simeq \boldsymbol{K}\left(\boldsymbol{R}-\frac{\boldsymbol{t} \boldsymbol{n}^{\mathrm{T}}}{d}\right) \boldsymbol{P} \\
    & \simeq \boldsymbol{K}\left(\boldsymbol{R}-\frac{\boldsymbol{t} \boldsymbol{n}^{\mathrm{T}}}{d}\right) \boldsymbol{K}^{-1} \boldsymbol{p}_{1}
    \end{aligned}
    $$
    
-   得到
    $$
    \boldsymbol{p}_2 \simeq \boldsymbol{H}\boldsymbol{p}_1
    
    \\
    
    \left(\begin{array}{c}
    u_{2} \\
    v_{2} \\
    1
    \end{array}\right) \simeq\left(\begin{array}{lll}
    h_{1} & h_{2} & h_{3} \\
    h_{4} & h_{5} & h_{6} \\
    h_{7} & h_{8} & h_{9}
    \end{array}\right)\left(\begin{array}{c}
    u_{1} \\
    v_{1} \\
    1
    \end{array}\right) .
    $$

-   自由度为8的单应矩阵可以通过4对不共线(三点不共线)的匹配特征点求解以下线性方程组
    $$
    \left(\begin{array}{cccccccc}
    u_{1}^{1} & v_{1}^{1} & 1 & 0 & 0 & 0 & -u_{1}^{1} u_{2}^{1} & -v_{1}^{1} u_{2}^{1} \\
    0 & 0 & 0 & u_{1}^{1} & v_{1}^{1} & 1 & -u_{1}^{1} v_{2}^{1} & -v_{1}^{1} v_{2}^{1} \\
    u_{1}^{2} & v_{1}^{2} & 1 & 0 & 0 & 0 & -u_{1}^{2} u_{2}^{2} & -v_{1}^{2} u_{2}^{2} \\
    0 & 0 & 0 & u_{1}^{2} & v_{1}^{2} & 1 & -u_{1}^{2} v_{2}^{2} & -v_{1}^{2} v_{2}^{2} \\
    u_{1}^{3} & v_{1}^{3} & 1 & 0 & 0 & 0 & -u_{1}^{3} u_{2}^{3} & -v_{1}^{3} u_{2}^{3} \\
    0 & 0 & 0 & u_{1}^{3} & v_{1}^{3} & 1 & -u_{1}^{3} v_{2}^{3} & -v_{1}^{3} v_{2}^{3} \\
    u_{1}^{4} & v_{1}^{4} & 1 & 0 & 0 & 0 & -u_{1}^{4} u_{2}^{4} & -v_{1}^{4} u_{2}^{4} \\
    0 & 0 & 0 & u_{1}^{4} & v_{1}^{4} & 1 & -u_{1}^{4} v_{2}^{4} & -v_{1}^{4} v_{2}^{4}
    \end{array}\right)\left(\begin{array}{l}
    h_{1} \\
    h_{2} \\
    h_{3} \\
    h_{4} \\
    h_{5} \\
    h_{6} \\
    h_{7} \\
    h_{8}
    \end{array}\right)=\left(\begin{array}{c}
    u_{2}^{1} \\
    v_{2}^{1} \\
    u_{2}^{2} \\
    v_{2}^{2} \\
    u_{2}^{3} \\
    v_{2}^{3} \\
    u_{2}^{4} \\
    v_{2}^{4}
    \end{array}\right)
    $$
    
-   求解单应矩阵同样需要分解得到 $\boldsymbol{R}, \boldsymbol{t}$, 同时通过判断正的深度值和已知平面法向量来得到最终运动估计
-   当特征点共面或相机发生纯旋转时, 基础矩阵的自由度下降, 即退化现象, 噪声会对多余的自由度起到决定作用. 一般会同时估计基础矩阵 $\boldsymbol{F}$ 与 单应矩阵 $\boldsymbol{H}$, 选择重投影误差小的作为最终的运动估计矩阵





### 实践: 2D-2D 对极约束求解相机运动

#### 尺度不确定性

-   对 $\boldsymbol{t}$ 的归一化导致了单目视觉的尺度不确定性
-   单目SLAM需要初始化, 两张图像必须有平移, 之后的轨迹与地图都以此平移为单位

#### 初始化的纯旋转问题

-   如果相机发生纯旋转, 导致 $\boldsymbol{t}$ 为零, 那么 $\boldsymbol{E}$ 也为零, 无法求解 $\boldsymbol{R}$, 此时可以根据 $\boldsymbol{H}$ 求得旋转, 但仍然无法估计特征点的空间位置, 所以单目初始化不能只有纯旋转, 必须要有平移.

#### 多于8点的情况

-   通过最小二乘求解
    $$
    \boldsymbol{A} \boldsymbol{e} = 0
    
    \\
    
    \min _{\boldsymbol{e}}\|\boldsymbol{A} \boldsymbol{e}\|_{2}^{2}=\min _{\boldsymbol{e}} \boldsymbol{e}^{\mathrm{T}} \boldsymbol{A}^{\mathrm{T}} \boldsymbol{A} \boldsymbol{e} .
    $$
    
-   当可能存在误匹配时, 使用RANSAC (随机采样一致性, Random Sample Concensus) 求解





### 三角测量

![image-20220718105425966](VSLAM14Chapter_note.assets/image-20220718105425966.png)

-   考虑图像 $I_{1}$ 和 $I_{2}$, 以左图为参考, 右图的变换矩阵为 $\boldsymbol{T}$ 。相机光心为 $O_{1}$ 和 $O_{2}$ 。在 $I_{1}$ 中有特征点 $p_{1}$, 对应 $I_{2}$ 中有特征点 $p_{2}$ 。理论上, 直线 $O_{1} p_{1}$ 与 $O_{2} p_{2}$ 在场景中会相交 于一点 $P$, 该点即两个特征点所对应的地图点在三维场景中的位置。然而由于噪声的影响, 这两条直线往往无法相交。因此, 可以通过最小二乘法求解。设 $\boldsymbol{x_1}, \boldsymbol{x_2}$ 为两个特征点的归一化坐标,
    $$
    s_{2} \boldsymbol{x}_{2}=s_{1} \boldsymbol{R} \boldsymbol{x}_{1}+\boldsymbol{t}
    
    \\
    
    s_{2} \boldsymbol{x}_{2}^{\wedge} \boldsymbol{x}_{2}=0=s_{1} \boldsymbol{x}_{2}^{\wedge} \boldsymbol{R} \boldsymbol{x}_{1}+\boldsymbol{x}_{2}^{\wedge} \boldsymbol{t}
    $$

    -   根据上式可以直接求解 $s_1$, $s_2$
    -   由于噪声, 更常见通过最小二乘求解

-   三角测量注意事项

    -   三角测量需要平移, 纯旋转无法得到三角测量
    -   平移量越大, 三角测量精度越高, 但会导致图像外观发生明显变化, 使得特征匹配变得困难, 称之为视差 (parallax) 问题
    -   单目视觉中, 通过几帧之后有足够平移的数据进行三角化, 称为延迟三角化






### 3D-2D: PnP法

#### PnP法简介

-   3D-2D PnP法 与 2D-2D 对极几何的比较 
    -   2D-2D 需要八个点以上才能估计相机位姿变化, 且存在初始化, 纯旋转, 尺度等问题. 如果两个图中其中一张图的3D位置已知, 最少只需要三对点就可以估计相机运动. 特征点的3D位置可以通过三角化或RGB-D相机深度值确定. 所以在双目或RGB-D相机的视觉里程计中, 可以直接使用3D-2D 的 PnP法估计相机运动.

-   PnP法的求解方法
    -   三对点估计位置 P3P
    -   直接线性变换 DLT
    -   EPnP (Efficient PnP)
    -   UPnP
    -   光束法平差 Bundle Adjustment BA



#### 直接线性变换法 DLT

-   问题: 已知一组3D点, 以及他们在相机中的投影位置, 求相机位姿

-   考虑某个空间点 $P$, 它的齐次坐标为 $\boldsymbol{P}=(X, Y, Z, 1)^{\mathrm{T}}$ 。在图像 $I_{1}$ 中, 投影到特征点 $\boldsymbol{x}_{1}=$ $\left(u_{1}, v_{1}, 1\right)^{\mathrm{T}}$ (以归一化平面齐次坐标表示)。此时, 相机的位姿 $\boldsymbol{R}, \boldsymbol{t}$ 是末知的。与单应矩阵的求解类似, 我们定义增广矩阵 $[\boldsymbol{R} \mid \boldsymbol{t}]$ 为一个 $3 \times 4$ 的矩阵, 包含了旋转与平移信息, 我们将其展开形式列写如下:
    $$
    s\left(\begin{array}{c}
    u_{1} \\
    v_{1} \\
    1
    \end{array}\right)=\left(\begin{array}{cccc}
    t_{1} & t_{2} & t_{3} & t_{4} \\
    t_{5} & t_{6} & t_{7} & t_{8} \\
    t_{9} & t_{10} & t_{11} & t_{12}
    \end{array}\right)\left(\begin{array}{l}
    X \\
    Y \\
    Z \\
    1
    \end{array}\right)
    
    \\
    \\
    
    u_{1}=\frac{t_{1} X+t_{2} Y+t_{3} Z+t_{4}}{t_{9} X+t_{10} Y+t_{11} Z+t_{12}}, \quad v_{1}=\frac{t_{5} X+t_{6} Y+t_{7} Z+t_{8}}{t_{9} X+t_{10} Y+t_{11} Z+t_{12}}
    
    \\
    \\
    
    \boldsymbol{t}_{1}=\left(t_{1}, t_{2}, t_{3}, t_{4}\right)^{\mathrm{T}}, \boldsymbol{t}_{2}=\left(t_{5}, t_{6}, t_{7}, t_{8}\right)^{\mathrm{T}}, \boldsymbol{t}_{3}=\left(t_{9}, t_{10}, t_{11}, t_{12}\right)^{\mathrm{T}}
    
    \\
    \\
    
    \boldsymbol{t}_{1}^{\mathrm{T}} \boldsymbol{P}-\boldsymbol{t}_{3}^{\mathrm{T}} \boldsymbol{P} u_{1}=0,\quad \boldsymbol{t}_{2}^{\mathrm{T}} \boldsymbol{P}-\boldsymbol{t}_{3}^{\mathrm{T}} \boldsymbol{P} v_{1}=0
    
    \\
    \\
    
    \left(\begin{array}{ccc}
    \boldsymbol{P}_{1}^{\mathrm{T}} & 0 & -u_{1} \boldsymbol{P}_{1}^{\mathrm{T}} \\
    0 & \boldsymbol{P}_{1}^{\mathrm{T}} & -v_{1} \boldsymbol{P}_{1}^{\mathrm{T}} \\
    \vdots & \vdots & \vdots \\
    \boldsymbol{P}_{N}^{\mathrm{T}} & 0 & -u_{N} \boldsymbol{P}_{N}^{\mathrm{T}} \\
    0 & \boldsymbol{P}_{N}^{\mathrm{T}} & -v_{N} \boldsymbol{P}_{N}^{\mathrm{T}}
    \end{array}\right)\left(\begin{array}{l}
    \boldsymbol{t}_{1} \\
    \boldsymbol{t}_{2} \\
    \boldsymbol{t}_{3}
    \end{array}\right)=0 .
    $$

-   $\boldsymbol{t}$ 一共12维, 最少通过6对匹配点实现矩阵 $\boldsymbol{T}$ 的线性求解, 称为直接线性转换DLT, 当匹配点大于6对时, 也可以使用SVD法对超定方程求最小二乘解

-   DLT求解时没有考虑 $\boldsymbol{T}$ 中元素互相之间的联系. 对于其中的旋转矩阵 $\boldsymbol{R}$, 我们要找出一个最好的旋转矩阵对它近似, 通过QR分解完成, 或通过以下方法:
    $$
    \boldsymbol{R} \leftarrow\left(\boldsymbol{R} \boldsymbol{R}^{\mathrm{T}}\right)^{-\frac{1}{2}} \boldsymbol{R}
    $$
    

####  P3P法

-   P3P 示意图

![image-20220725120713009](VSLAM14Chapter_note.assets/image-20220725120713009.png)

-   推导

$$
\Delta O a b-\Delta O A B, \quad \Delta O b c-\Delta O B C, \quad \Delta O a c-\Delta O A C

\\
\\

\begin{aligned}
&O A^{2}+O B^{2}-2 O A \cdot O B \cdot \cos \langle a, b\rangle=A B^{2} \\
&O B^{2}+O C^{2}-2 O B \cdot O C \cdot \cos \langle b, c\rangle=B C^{2} \\
&O A^{2}+O C^{2}-2 O A \cdot O C \cdot \cos \langle a, c\rangle=A C^{2}
\end{aligned}

\\
\\

x=O A / O C, y=O B / O C

\\
\\

\begin{aligned}
&x^{2}+y^{2}-2 x y \cos \langle a, b\rangle=A B^{2} / O C^{2} \\
&y^{2}+1^{2}-2 y \cos \langle b, c\rangle=B C^{2} / O C^{2} \\
&x^{2}+1^{2}-2 x \cos \langle a, c\rangle=A C^{2} / O C^{2}
\end{aligned}

\\
\\

v=A B^{2} / O C^{2}, u v=B C^{2} / O C^{2}, w v=A C^{2} / O C^{2}

\\
\\

\begin{aligned}
&x^{2}+y^{2}-2 x y \cos \langle a, b\rangle-v=0 \\
&y^{2}+1^{2}-2 y \cos \langle b, c\rangle-u v=0 \\
&x^{2}+1^{2}-2 x \cos \langle a, c\rangle-w v=0
\end{aligned}

\\
\\

\begin{aligned}
&(1-u) y^{2}-u x^{2}-\cos \langle b, c\rangle y+2 u x y \cos \langle a, b\rangle+1=0 \\
&(1-w) x^{2}-w y^{2}-\cos \langle a, c\rangle x+2 w x y \cos \langle a, b\rangle+1=0
\end{aligned}
$$



-   已知量
    -   通过2D图像位置获取三个余弦角
        -    $\cos{<a, b>},\cos{<b,c>},\cos{<a,c>}$
    -   通过世界坐标系获取 $u, w$
        -   $u=B C^{2} / A B^{2}, w=A C^{2} / A B^{2}$

-   通过吴消元法获得P3P的解析解, 最多可能有4个解, 通过验证点来计算最可能的解
-   缺点
    -   P3P只能利用三个点的信息, 当配对点多于三组, 难以利用更多信息
    -   3D 或 2D点受噪声影响或误匹配, 算法失效
-   SLAM 中通过P3P/EPnP 估计相机位姿, 然后构建最小二乘优化问题进行优化, 即Bundle Adjustment





#### 最小化重投影误差求解PnP (BA, Bundle Adjustment)

-   把相机位姿和空间点的三维坐标放在一起进行最小化的问题, 统称为 Bundle Adjustment. 

    ![image-20220725151506151](VSLAM14Chapter_note.assets/image-20220725151506151.png)

-   构建最小二乘问题, 寻找最好的相机位姿 $\boldsymbol{T}$
    $$
    s_{i}\left[\begin{array}{c}u_{i} \\ v_{i} \\ 1\end{array}\right]=\boldsymbol{K} \boldsymbol{T}\left[\begin{array}{c}X_{i} \\ Y_{i} \\ Z_{i} \\ 1\end{array}\right]
    
    \\
    \\
    
    s_{i} \boldsymbol{u}_{i}=\boldsymbol{K} \boldsymbol{T} \boldsymbol{P}_{i}
    
    \\
    \\
    
    \boldsymbol{T}^{*}=\arg \min _{\boldsymbol{T}} \frac{1}{2} \sum_{i=1}^{n}\left\|\boldsymbol{u}_{i}-\frac{1}{s_{i}} \boldsymbol{K} \boldsymbol{T} \boldsymbol{P}_{i}\right\|_{2}^{2}
    $$

-   误差项关于优化变量的导数
    $$
    \boldsymbol{e}(\boldsymbol{x}+\Delta \boldsymbol{x}) \approx \boldsymbol{e}(\boldsymbol{x}) + \boldsymbol{J}^{\mathrm{T}} \Delta \boldsymbol{x}
    $$

-   重投影误差项关于相机位姿李代数的导数 $\frac{\part \boldsymbol{e}}{\part{\delta \boldsymbol{\xi}}}$ 
    $$
    \boldsymbol{P}^{\prime}=(\boldsymbol{T} \boldsymbol{P})_{1: 3}=\left[X^{\prime}, Y^{\prime}, Z^{\prime}\right]^{\mathrm{T}}
    
    \\
    \\
    
    \boldsymbol{s} \boldsymbol{u} = \boldsymbol{K} \boldsymbol{P}^{\prime}
    
    \\
    \\
    
    \left[\begin{array}{l}
    s u \\
    s v \\
    s
    \end{array}\right]=\left[\begin{array}{ccc}
    f_{x} & 0 & c_{x} \\
    0 & f_{y} & c_{y} \\
    0 & 0 & 1
    \end{array}\right]\left[\begin{array}{l}
    X^{\prime} \\
    Y^{\prime} \\
    Z^{\prime}
    \end{array}\right]
    
    \\
    \\
    
    u=f_{x} \frac{X^{\prime}}{Z^{\prime}}+c_{x}, \quad v=f_{y} \frac{Y^{\prime}}{Z^{\prime}}+c_{y} 
    
    \\
    \\
    
    \frac{\partial \boldsymbol{e}}{\partial \delta \boldsymbol{\xi}}=\lim _{\delta \boldsymbol{\xi} \rightarrow 0} \frac{\boldsymbol{e}(\delta \boldsymbol{\xi} \oplus \boldsymbol{\xi})-\boldsymbol{e}(\boldsymbol{\xi})}{\delta \boldsymbol{\xi}}=\frac{\partial \boldsymbol{e}}{\partial \boldsymbol{P}^{\prime}} \frac{\partial \boldsymbol{P}^{\prime}}{\partial \delta \boldsymbol{\xi}}
    
    \\
    \\
    
    \frac{\partial \boldsymbol{e}}{\partial \boldsymbol{P}^{\prime}}=-\left[\begin{array}{ccc}
    \frac{\partial u}{\partial X^{\prime}} & \frac{\partial u}{\partial Y^{\prime}} & \frac{\partial u}{\partial Z^{\prime}} \\
    \frac{\partial v}{\partial X^{\prime}} & \frac{\partial v}{\partial Y^{\prime}} & \frac{\partial v}{\partial Z^{\prime}}
    \end{array}\right]=-\left[\begin{array}{ccc}
    \frac{f_{x}}{Z^{\prime}} & 0 & -\frac{f_{x} X^{\prime}}{Z^{\prime 2}} \\
    0 & \frac{f_{y}}{Z^{\prime}} & -\frac{f_{y} Y^{\prime}}{Z^{\prime 2}}
    \end{array}\right]
    
    \\
    \\
    
    \frac{\partial(\boldsymbol{T} \boldsymbol{P})}{\partial \delta \boldsymbol{\xi}}=(\boldsymbol{T} \boldsymbol{P})^{\odot}=\left[\begin{array}{cc}
    \boldsymbol{I} & -\boldsymbol{P}^{\prime \wedge} \\
    \mathbf{0}^{\mathrm{T}} & \mathbf{0}^{\mathrm{T}}
    \end{array}\right]
    
    \\
    \\
    
    \frac{\partial \boldsymbol{P}^{\prime}}{\partial \delta \boldsymbol{\xi}}=\left[\boldsymbol{I},-\boldsymbol{P}^{\prime \wedge}\right]
    
    \\
    \\
    
    \frac{\partial \boldsymbol{e}}{\partial \delta \boldsymbol{\xi}}=-\left[\begin{array}{cccccc}
    \frac{f_{x}}{Z^{\prime}} & 0 & -\frac{f_{x} X^{\prime}}{Z^{\prime 2}} & -\frac{f_{x} X^{\prime} Y^{\prime}}{Z^{\prime 2}} & f_{x}+\frac{f_{x} X^{\prime 2}}{Z^{\prime 2}} & -\frac{f_{x} Y^{\prime}}{Z^{\prime}} \\
    0 & \frac{f_{y}}{Z^{\prime}} & -\frac{f_{y} Y^{\prime}}{Z^{\prime 2}} & -f_{y}-\frac{f_{y} Y^{\prime 2}}{Z^{\prime 2}} & \frac{f_{y} X^{\prime} Y^{\prime}}{Z^{\prime 2}} & \frac{f_{y} X^{\prime}}{Z^{\prime}}
    \end{array}\right]
    $$

-   重投影误差项关于空间点 $\boldsymbol{P}$ 的导数 $\frac{\part{\boldsymbol{e}}}{\part \boldsymbol{P}}$ 
    $$
    \frac{\partial \boldsymbol{e}}{\partial \boldsymbol{P}}=\frac{\partial \boldsymbol{e}}{\partial \boldsymbol{P}^{\prime}} \frac{\partial \boldsymbol{P}^{\prime}}{\partial \boldsymbol{P}}
    
    \\
    \\
    
    \boldsymbol{P}^{\prime}=(\boldsymbol{T} \boldsymbol{P})_{1: 3}=\boldsymbol{R} \boldsymbol{P}+\boldsymbol{t}
    
    \\
    \\
    
    \frac{\partial \boldsymbol{e}}{\partial \boldsymbol{P}}=-\left[\begin{array}{ccc}
    \frac{f_{x}}{Z^{\prime}} & 0 & -\frac{f_{x} X^{\prime}}{Z^{\prime 2}} \\
    0 & \frac{f_{y}}{Z^{\prime}} & -\frac{f_{y} Y^{\prime}}{Z^{\prime 2}}
    \end{array}\right] \boldsymbol{R}
    $$
    

### 实践: 求解3D-2D PnP

#### 通过 OpenCV 求解EPnP

#### 通过手写高斯牛顿法进行BA优化

$$
\boldsymbol{T}^{*}=\arg \min _{\boldsymbol{T}} \frac{1}{2} \sum_{i=1}^{n}\left\|\boldsymbol{u}_{i}-\frac{1}{s_{i}} \boldsymbol{K} \boldsymbol{T} \boldsymbol{P}_{i}\right\|_{2}^{2}

\\
\\

\boldsymbol{e}(\boldsymbol{x}) = \sum_{i=1}^{n}\left\|\boldsymbol{u}_{i}-\frac{1}{s_{i}} \boldsymbol{K} \boldsymbol{T} \boldsymbol{P}_{i}\right\|_{2}^{2}

\\
\\

\boldsymbol{e}(\boldsymbol{x}+\Delta \boldsymbol{x}) \approx \boldsymbol{e}(\boldsymbol{x})+\boldsymbol{J}^{\mathrm{T}} \Delta \boldsymbol{x}

\\
\\

\boldsymbol{J}^{\text{T}} =  \frac{\partial \boldsymbol{e}}{\partial \delta \boldsymbol{\xi}}=-\left[\begin{array}{ccrrrr}
\frac{f_{x}}{Z^{\prime}} & 0 & -\frac{f_{x} X^{\prime}}{Z^{\prime 2}} & -\frac{f_{x} X^{\prime} Y^{\prime}}{Z^{\prime 2}} & f_{x}+\frac{f_{x} X^{\prime 2}}{Z^{\prime 2}} & -\frac{f_{x} Y^{\prime}}{Z^{\prime}} \\
0 & \frac{f_{y}}{Z^{\prime}} & -\frac{f_{y} Y^{\prime}}{Z^{\prime 2}} & -f_{y}-\frac{f_{y} Y^{\prime 2}}{Z^{\prime 2}} & \frac{f_{y} X^{\prime} Y^{\prime}}{Z^{\prime 2}} & \frac{f_{y} X^{\prime}}{Z^{\prime}}
\end{array}\right] 

\\
\\

\boldsymbol{J}\boldsymbol{J}^{\text{T}} \Delta \boldsymbol{x} = - \boldsymbol{J} \boldsymbol{e}(\boldsymbol{x})

\\
\\
$$

#### 通过G2O 进行BA优化

-   顶点: 第二个相机的位姿节点 $\boldsymbol{T} \in \mathrm{SE}(3)$ 。需要实现顶点的更新
2. 边: 每个 $3 \mathrm{D}$ 点在第二个相机中的投影, 以观测方程来描述如下. 需要实现边的误差计算和雅可比矩阵.

$$
\boldsymbol{z}_{j}=h\left(\boldsymbol{T}, \boldsymbol{P}_{j}\right) .
$$

![image-20220725155331447](VSLAM14Chapter_note.assets/image-20220725155331447.png)







### 3D-3D: ICP法 (Iterative Closest Point)

-   问题描述: 对一组匹配好的3D点寻找欧氏变换
    $$
    \forall i, \boldsymbol{p}_{i}=\boldsymbol{R} \boldsymbol{p}_{i}^{\prime}+\boldsymbol{t}
    $$
    



#### SVD法求解ICP

-   误差项
    $$
    \boldsymbol{e}_{i}=\boldsymbol{p}_{i}-\left(\boldsymbol{R} \boldsymbol{p}_{i}^{\prime}+\boldsymbol{t}\right)
    $$

-   构建关于 $\boldsymbol{R}$ 的最小二乘问题

$$
\min _{\boldsymbol{R}, \boldsymbol{t}} \frac{1}{2} \sum_{i=1}^{n}\left\|\left(\boldsymbol{p}_{i}-\left(\boldsymbol{\boldsymbol { R } _ { \boldsymbol { i } }}{ }^{\prime}+\boldsymbol{t}\right)\right)\right\|_{2}^{2}

\\
\\

\boldsymbol{p}=\frac{1}{n} \sum_{i=1}^{n}\left(\boldsymbol{p}_{i}\right), \quad \boldsymbol{p}^{\prime}=\frac{1}{n} \sum_{i=1}^{n}\left(\boldsymbol{p}_{i}^{\prime}\right)

\\
\\

\min _{\boldsymbol{R}, \boldsymbol{t}} J=\frac{1}{2} \sum_{i=1}^{n}\left\|\boldsymbol{p}_{i}-\boldsymbol{p}-\boldsymbol{R}\left(\boldsymbol{p}_{i}^{\prime}-\boldsymbol{p}^{\prime}\right)\right\|^{2}+\left\|\boldsymbol{p}-\boldsymbol{R} \boldsymbol{p}^{\prime}-\boldsymbol{t}\right\|^{2}

\\
\\

\boldsymbol{q}_{i}=\boldsymbol{p}_{i}-\boldsymbol{p}, \quad \boldsymbol{q}_{i}^{\prime}=\boldsymbol{p}_{i}^{\prime}-\boldsymbol{p}^{\prime}

\\
\\

\boldsymbol{R}^{*}=\arg \min _{\boldsymbol{R}} \frac{1}{2} \sum_{i=1}^{n}\left\|\boldsymbol{q}_{i}-\boldsymbol{R} \boldsymbol{q}_{i}^{\prime}\right\|^{2}

\\
\\

\boldsymbol{t}^{*}=\boldsymbol{p}-\boldsymbol{R} \boldsymbol{p}^{\prime}

\\
\\

\frac{1}{2} \sum_{i=1}^{n}\left\|\boldsymbol{q}_{i}-\boldsymbol{R} \boldsymbol{q}_{i}^{\prime}\right\|^{2}=\frac{1}{2} \sum_{i=1}^{n}\left(\boldsymbol{q}_{i}^{\mathrm{T}} \boldsymbol{q}_{i}+\boldsymbol{q}_{i}^{\prime \mathrm{T}} \boldsymbol{R}^{\mathrm{T}} \boldsymbol{R} \boldsymbol{q}_{i}^{\prime}-2 \boldsymbol{q}_{i}^{\mathrm{T}} \boldsymbol{R} \boldsymbol{q}_{i}^{\prime}\right)

\\
\\

\sum_{i=1}^{n}-\boldsymbol{q}_{i}^{\mathrm{T}} \boldsymbol{R} \boldsymbol{q}_{i}^{\prime}=\sum_{i=1}^{n}-\operatorname{tr}\left(\boldsymbol{R} \boldsymbol{q}_{i}^{\prime} \boldsymbol{q}_{i}^{\mathrm{T}}\right)=-\operatorname{tr}\left(\boldsymbol{R} \sum_{i=1}^{n} \boldsymbol{q}_{i}^{\prime} \boldsymbol{q}_{i}^{\mathrm{T}}\right)
$$

-   通过SVD分解求解 $\boldsymbol{R}$
    $$
    \boldsymbol{W}=\sum_{i=1}^{n} \boldsymbol{q}_{i} \boldsymbol{q}_{i}^{\prime \mathrm{T}}
    
    \\
    \\
    
    \boldsymbol{W}=\boldsymbol{U} \boldsymbol{\Sigma} \boldsymbol{V}^{\mathrm{T}}
    
    \\
    \\
    
    \boldsymbol{R}=\boldsymbol{U} \boldsymbol{V}^{\mathrm{T}}
    $$

-   当 $\boldsymbol{R}$ 的行列式为负, 则取 $-\boldsymbol{R}$

-   根据下式求 $\boldsymbol{t}$
    $$
    \boldsymbol{t}^{*}=\boldsymbol{p}-\boldsymbol{R} \boldsymbol{p}^{\prime}
    $$
    

#### 非线性优化法求解ICP

-   3D-3D的非线性优化问题类似3D-2D 求解PnP问题

$$
\min _{\boldsymbol{\xi}}=\frac{1}{2} \sum_{i=1}^{n}\left\|\left(\boldsymbol{p}_{i}-\exp \left(\boldsymbol{\xi}^{\wedge}\right) \boldsymbol{p}_{i}^{\prime}\right)\right\|_{2}^{2}

\\
\\

\frac{\partial \boldsymbol{e}}{\partial \delta \boldsymbol{\xi}}=-\left(\exp \left(\boldsymbol{\xi}^{\wedge}\right) \boldsymbol{p}_{i}^{\prime}\right)^{\odot}
$$



-   当深度已知时, 建模3D-3D误差, 当深度未知时, 建模3D-2D误差, 从而将3D-3D 与 3D-2D放在同一个优化问题中解决

