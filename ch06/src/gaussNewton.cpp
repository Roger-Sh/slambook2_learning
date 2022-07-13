#include <iostream>
#include <chrono>   // time
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>


/**
 * @brief gaussNewton
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    // 初始化参数列表
    double ar = 1.0, br = 2.0, cr = 1.0;    // 真实参数值
    double ae = 2.0, be = -1.0, ce = 5.0;   // 初始估计参数值

    // 初始化数据噪声
    int N = 100;                            // 数据点
    double w_sigma = 1.0;                   // 噪声Sigma值
    double inv_sigma = 1.0 / w_sigma;       // 噪声倒数
    cv::RNG rng;                            // OpenCV随机数产生器
    std::vector<double> x_data, y_data; // 数据
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    // 开始Gauss-Newton迭代
    int iterations = 100;          // 迭代次数
    double cost = 0, lastCost = 0; // 本次迭代的cost和上一次迭代的cost
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();    // timer
    for (int iter = 0; iter < iterations; iter++)
    {
        // init H, b
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero(); // Hessian = J^T W^{-1} J in Gauss-Newton
        Eigen::Vector3d b = Eigen::Vector3d::Zero(); // bias
        cost = 0;

        for (int i = 0; i < N; i++)
        {
            // 第i个数据点
            double xi = x_data[i], yi = y_data[i]; 
            
            // 当前模型的误差
            double error = yi - exp(ae * xi * xi + be * xi + ce);
            
            // 雅可比矩阵
            Eigen::Vector3d J;                                  
            J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce); // de/da
            J[1] = -xi * exp(ae * xi * xi + be * xi + ce);      // de/db
            J[2] = -exp(ae * xi * xi + be * xi + ce);           // de/dc

            // update H, b
            H += inv_sigma * inv_sigma * J * J.transpose();     
            b += -inv_sigma * inv_sigma * error * J;

            // update cost
            cost += error * error;
        }

        // 求解线性方程 Hx=b, cholesky法
        Eigen::Vector3d dx = H.ldlt().solve(b);
        if (isnan(dx[0]))
        {
            std::cout << "result is nan!" << std::endl;
            break;
        }

        // break if current cost is bigger than last cost
        if (iter > 0 && cost >= lastCost)
        {
            std::cout << "cost: " << cost << " >= last cost: " << lastCost << ", break." << std::endl;
            break;
        }

        // update parameter
        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        // update lastcost
        lastCost = cost;

        std::cout << "total cost: " << cost << ", \t\tupdate: " << dx.transpose() << "\t\testimated params: " << ae << "," << be << "," << ce << std::endl;
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;

    std::cout << "estimated abc = " << ae << ", " << be << ", " << ce << std::endl;
    return 0;
}
