#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>

/**
 * @brief 曲线优化模型的顶点，即待优化参数，模板参数：优化变量维度和数据类型
 *
 */
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    // Eigen 重写 Operator New 来保证内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 顶点重置函数
    // 这里 子类覆写父类的虚函数，也需要加上 virtual 和 override
    // 是为了让编译器来检查我们又没有正确覆写父类的虚函数，并提醒读代码的人这是个覆写的虚函数
    virtual void setToOriginImpl() override
    {
        // 简单将估计值置为0
        _estimate << 0, 0, 0;
    }

    // 顶点更新函数
    // 某些问题中优化变量不一定通过加法，比如优化相机位姿时，可能通过左乘更新
    virtual void oplusImpl(const double *update) override
    {
        _estimate += Eigen::Vector3d(update);
    }

    // 存盘和读盘：留空
    virtual bool read(std::istream &in) {}

    virtual bool write(std::ostream &out) const {}
};

/**
 * @brief 曲线优化模型的边, 即误差模型，模板参数：观测值维度，类型，连接顶点类型
 *
 */
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
    // Eigen 重写 Operator New 来保证内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 构造器
    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

    // 边的误差计算函数
    // 需要取出边所连接的顶点的当前估计值
    // 计算曲线模型误差函数
    virtual void computeError() override
    {
        // 取顶点0，将其强制转换成自定义的顶点
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);

        // 通过该顶点预测当前参数
        const Eigen::Vector3d abc = v->estimate();

        // 计算误差
        // y-exp(ax^2+bx+c)
        _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
    }

    // 边的计算雅可比矩阵函数
    // 每条边相对于顶点的雅可比
    virtual void linearizeOplus() override
    {
        // 取顶点0，将其强制转换成自定义的顶点
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);

        // 通过该顶点预测当前参数
        const Eigen::Vector3d abc = v->estimate();

        // 计算雅可比矩阵
        double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
        _jacobianOplusXi[0] = -_x * _x * y;
        _jacobianOplusXi[1] = -_x * y;
        _jacobianOplusXi[2] = -y;
    }

    virtual bool read(std::istream &in) {}

    virtual bool write(std::ostream &out) const {}

public:
    double _x;  // x 值， y 值为 _measurement
};

/**
 * @brief 本程序演示了通过 G2O 构建因子图，解决曲线拟合问题。
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    // 初始化参数
    double ar = 1.0, br = 2.0, cr = 1.0;   // 真实参数值
    double ae = 2.0, be = -1.0, ce = 5.0;  // 估计参数值

    // 初始化100个带随机噪声的数据
    int N = 100;           // 数据点
    double w_sigma = 1.0;  // 噪声Sigma值
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;                         // OpenCV随机数产生器
    std::vector<double> x_data, y_data;  // 数据
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;            // 每个误差项优化变量维度为3，误差值维度为1
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;  // 线性求解器类型

    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;  // 图模型
    optimizer.setAlgorithm(solver);  // 设置求解器
    optimizer.setVerbose(true);      // 打开调试输出

    // 往图中增加顶点，目前只有一个顶点
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae, be, ce));
    v->setId(0);
    optimizer.addVertex(v);

    // 往图中增加边，有几个数据，就有几个边
    for (int i = 0; i < N; i++)
    {
        CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);                                                                    // 设置连接的顶点
        edge->setMeasurement(y_data[i]);                                                          // 观测数值
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));  // 信息矩阵：协方差矩阵之逆
        optimizer.addEdge(edge);
    }

    // 执行优化
    std::cout << "start optimization" << std::endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);  // 设置步数
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;

    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
    std::cout << "estimated model: " << abc_estimate.transpose() << std::endl;

    return 0;
}