#include <ceres/ceres.h>

#include <chrono>  // timer
#include <iostream>
#include <opencv2/core/core.hpp>

/**
 * @brief 残差块：代价函数的计算模型
 *
 */
struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

    /**
     * @brief 残差的计算
     *
     * @tparam T
     * @param abc
     * @param residual
     * @return true
     * @return false
     */
    template <typename T>
    bool operator()(
        const T *const abc,  // 模型参数，有3维
        T *residual          // 返回的残差值
    ) const                  // 函数括号后面加const,表示不能修改类中成员变量
    {
        // y-exp(ax^2+bx+c)
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }

    // x,y数据
    const double _x, _y;
};

/**
 * @brief 本程序演示了通过 ceres 解决曲线拟合的优化问题
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    // 初始化优化参数
    double ar = 1.0, br = 2.0, cr = 1.0;   // 真实参数值
    double ae = 2.0, be = -1.0, ce = 5.0;  // 估计参数值

    // 初始化100个数据
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

    // 参数块
    double abc[3] = {ae, be, ce};

    // 构建最小二乘问题
    ceres::Problem problem;
    for (int i = 0; i < N; i++)
    {
        // 残差块
        // 使用自动求导，模板参数：(误差类型，输出维度，输入维度)，维数要与前面struct中一致
        // 这里需要手动设置残差块（代价函数）的计算方式
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(new CURVE_FITTING_COST(x_data[i], y_data[i])),  // 残差块
            nullptr,  // 核函数，这里不使用，为空
            abc       // 参数块
        );
    }

    // 配置求解器
    ceres::Solver::Options options;                             // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;                // 输出到cout

    // 求解
    ceres::Solver::Summary summary;  // 优化信息
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);  // 开始优化
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;

    // 输出结果
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "estimated a,b,c = ";
    for (auto a : abc)
        std::cout << a << " ";
    std::cout << std::endl;

    return 0;
}