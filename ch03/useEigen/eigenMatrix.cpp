#include <iostream>
#include <ctime>

// Eigen
#include <Eigen/Core>  // eigen 核心部分
#include <Eigen/Dense> // eigen 稠密计算部分

#define MATRIX_SIZE 50

/****************************
* 本程序演示了 Eigen 基本类型的使用
****************************/

int main(int argc, char **argv)
{
     // Eigen 声明
     Eigen::Matrix<float, 2, 3> matrix_23;
     Eigen::Vector3d v_3d;
     Eigen::Matrix<float, 3, 1> vd_3d;
     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
     Eigen::MatrixXd matrix_x;

     // 初始化
     Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero(); 
     matrix_23 << 1, 2, 3, 4, 5, 6;

     // 输出
     std::cout << "matrix 2x3 from 1 to 6: \n"
               << matrix_23 << std::endl;

     // 用()访问矩阵中的元素
     std::cout << "print matrix 2x3: " << std::endl;
     for (int i = 0; i < 2; i++)
     {
          for (int j = 0; j < 3; j++)
          {
               std::cout << matrix_23(i, j) << "\t";
          }
          std::cout << std::endl;
     }

     // 矩阵和向量相乘（实际上仍是矩阵和矩阵）
     v_3d << 3, 2, 1;
     vd_3d << 4, 5, 6;

     // Eigen 应该显式转换数据类型
     Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
     std::cout << "[1,2,3;4,5,6]*[3,2,1]=" << result.transpose() << std::endl;

     Eigen::Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
     std::cout << "[1,2,3;4,5,6]*[4,5,6]: " << result2.transpose() << std::endl;

     // 同样你不能搞错矩阵的维度
     // 试着取消下面的注释，看看Eigen会报什么错
     // Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

     // 一些矩阵运算
     // 随机数矩阵
     matrix_33 = Eigen::Matrix3d::Random(); 
     std::cout << "random matrix: \n"
          << matrix_33 << std::endl;
     // 转置
     std::cout << "transpose: \n"
          << matrix_33.transpose() << std::endl;      
     // 各元素和    
     std::cout << "sum: " << matrix_33.sum() << std::endl;     
     // 迹
     std::cout << "trace: " << matrix_33.trace() << std::endl; 
     // 数乘
     std::cout << "times 10: \n"
          << 10 * matrix_33 << std::endl; 
     // 逆
     std::cout << "inverse: \n"
          << matrix_33.inverse() << std::endl;    
     // 行列式            
     std::cout << "det: " << matrix_33.determinant() << std::endl; 
     // 特征值
     // 实对称矩阵可以保证对角化成功
     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
     std::cout << "Eigen values = \n"
          << eigen_solver.eigenvalues() << std::endl;
     std::cout << "Eigen vectors = \n"
          << eigen_solver.eigenvectors() << std::endl;

     // 解方程
     // 我们求解 matrix_NN * x = v_Nd 这个方程
     // N的大小在前边的宏里定义，它由随机数生成
     // 直接求逆自然是最直接的，但是求逆运算量大
     Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
     matrix_NN = matrix_NN * matrix_NN.transpose(); // 保证半正定
     Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

     clock_t time_stt = clock(); // 计时
     // 直接求逆
     Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
     std::cout << "time of normal inverse is "
          << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << std::endl;
     std::cout << "x = " << x.transpose() << std::endl;

     // 通常用矩阵分解来求，例如QR分解，速度会快很多
     time_stt = clock();
     x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
     std::cout << "time of Qr decomposition is "
          << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << std::endl;
     std::cout << "x = " << x.transpose() << std::endl;

     // 对于正定矩阵，还可以用cholesky分解来解方程
     time_stt = clock();
     x = matrix_NN.ldlt().solve(v_Nd);
     std::cout << "time of ldlt decomposition is "
          << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << std::endl;
     std::cout << "x = " << x.transpose() << std::endl;

     return 0;
}