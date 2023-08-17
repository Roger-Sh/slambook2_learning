# 视觉SLAM十四讲笔记

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

- **罗德里格斯公式**

    - 从旋转向量 $\theta \boldsymbol{n}$ 到旋转矩阵 $\boldsymbol{R}$

    $$
    \boldsymbol{R} = \cos(\theta )\boldsymbol{I} + (1-\cos(\theta))\boldsymbol{n}\boldsymbol{n}^{\mathrm{T}} + \sin(\theta)\boldsymbol{n}^{\land}
    $$

    - 从旋转矩阵 $\boldsymbol{R}$ 到旋转向量 $\theta \boldsymbol{n}$， 通过对 $\boldsymbol{R}$ 求迹（即求矩阵对角线元素之和）：

    $$
    \theta = \arccos(\frac{(\mathrm{tr}(\boldsymbol{R})) - 1}{2}) \\
    \theta = \arccos(\frac{(1+2\cos(\theta)) - 1}{2}) \\
    \boldsymbol{R}\boldsymbol{n} 
    = \boldsymbol{n}, (\boldsymbol{R}-\bold{I})\boldsymbol{n} = \boldsymbol{0}
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

        





### CPP Demo

-   useEigen
    -   使用 Eigen 库的例子
-   useGeometry
    -   使用 Eigen 中的几何库的例子
-   visualizeGeometry
    -   使用 pangolin 库进行可视化的例子
-   examples
    -   使用 pangolin 库可视化预先储存的轨迹
