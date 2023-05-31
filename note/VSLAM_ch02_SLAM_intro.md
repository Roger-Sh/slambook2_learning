# 视觉SLAM十四讲笔记

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



### CPP-Demo

-   helloSLAM.cpp
    -   simple cpp start demo
-   useHello.cpp
    -   a cpp shared lib demo

