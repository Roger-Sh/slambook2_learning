#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <Eigen/Core>
#include <chrono>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sophus/se3.hpp>

std::string img1_path = "../../../chapter_demo/ch07/image/1.png";
std::string img2_path = "../../../chapter_demo/ch07/image/2.png";
std::string depth_img1_path = "../../../chapter_demo/ch07/image/1_depth.png";
std::string depth_img2_path = "../../../chapter_demo/ch07/image/2_depth.png";

/**
 * @brief find feature matches
 *
 * @param img_1
 * @param img_2
 * @param keypoints_1
 * @param keypoints_2
 * @param matches
 */
void find_feature_matches(
    const cv::Mat &img_1,
    const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches);

/**
 * @brief 像素坐标转相机归一化坐标
 *
 * @param p
 * @param K
 * @return cv::Point2d
 */
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);

/**
 * @brief typedef for 2d, 3d Eigen vector
 *
 */
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

/**
 * @brief BA by g2o
 *
 * @param points_3d, 3d pts from img1
 * @param points_2d, 2d pts from img2
 * @param K, camera intrinsic matrix
 * @param pose, cam pose between img1 and img2
 */
void bundleAdjustmentG2O(const VecVector3d &points_3d, const VecVector2d &points_2d, const cv::Mat &K, Sophus::SE3d &pose);

/**
 * @brief BA by gauss-newton
 *
 * @param points_3d, 3d pts from img1
 * @param points_2d, 2d pts from img2
 * @param K, camera intrinsic matrix
 * @param pose, cam pose between img1 and img2
 */
void bundleAdjustmentGaussNewton(const VecVector3d &points_3d, const VecVector2d &points_2d, const cv::Mat &K, Sophus::SE3d &pose);

/**
 * @brief main, different methods to solve 3D-2D PnP problem
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    // read imgs
    cv::Mat img_1 = cv::imread(img1_path, CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(img2_path, CV_LOAD_IMAGE_COLOR);
    assert(img_1.data && img_2.data && "Can not load images!");

    // find ORB feature matches
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    std::cout << "一共找到了" << matches.size() << "组匹配点" << std::endl;

    // build 3d point from 1_depth
    // 3d from img1, 2d from img2, calc T between img1, img2
    cv::Mat d1 = cv::imread(depth_img1_path, CV_LOAD_IMAGE_UNCHANGED);  // 深度图为16位无符号数，单通道图像
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<cv::Point3f> pts_3d;
    std::vector<cv::Point2f> pts_2d;
    for (cv::DMatch m : matches)
    {
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if (d == 0)  // bad depth
            continue;
        float dd = d / 5000.0;
        cv::Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        pts_3d.push_back(cv::Point3f(p1.x * dd, p1.y * dd, dd));
        pts_2d.push_back(keypoints_2[m.trainIdx].pt);
    }
    std::cout << "3d-2d pairs: " << pts_3d.size() << std::endl;

    // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    std::cout << "solve pnp in opencv" << std::endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    cv::Mat r, t;
    solvePnP(pts_3d, pts_2d, K, cv::Mat(), r, t, false);
    cv::Mat R;
    cv::Rodrigues(r, R);  // r为旋转向量形式，用Rodrigues公式转换为矩阵
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve pnp in opencv cost time: " << time_used.count() << " seconds." << std::endl;
    std::cout << "R=" << std::endl << R << std::endl;
    std::cout << "t=" << std::endl << t << std::endl;

    // prepare 3d-2d vector with eigen
    VecVector3d pts_3d_eigen;
    VecVector2d pts_2d_eigen;
    for (size_t i = 0; i < pts_3d.size(); ++i)
    {
        pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
        pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
    }

    // BA by gauss newton
    std::cout << "calling bundle adjustment by gauss newton" << std::endl;
    Sophus::SE3d pose_gn;
    t1 = std::chrono::steady_clock::now();
    bundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve pnp by gauss newton cost time: " << time_used.count() << " seconds." << std::endl;

    // BA by G2O
    std::cout << "calling bundle adjustment by g2o" << std::endl;
    Sophus::SE3d pose_g2o;
    t1 = std::chrono::steady_clock::now();
    bundleAdjustmentG2O(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << std::endl;
    return 0;
}

/**
 * @brief find feature matches
 *
 * @param img_1
 * @param img_2
 * @param keypoints_1
 * @param keypoints_2
 * @param matches
 */
void find_feature_matches(
    const cv::Mat &img_1,
    const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches)
{
    //-- 初始化
    cv::Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);

    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    // 找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = match[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= std::max(2 * min_dist, 30.0))
        {
            matches.push_back(match[i]);
        }
    }
}

/**
 * @brief 像素坐标转相机归一化坐标
 *
 * @param p
 * @param K
 * @return cv::Point2d
 */
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K)
{
    return cv::Point2d((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0), (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

/**
 * @brief BA by gauss-newton
 *
 * @param points_3d, 3d pts from img1
 * @param points_2d, 2d pts from img2
 * @param K, camera intrinsic matrix
 * @param pose, cam pose between img1 and img2
 */
void bundleAdjustmentGaussNewton(const VecVector3d &points_3d, const VecVector2d &points_2d, const cv::Mat &K, Sophus::SE3d &pose)
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    const int iterations = 10;
    double cost = 0, lastCost = 0;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    // loop for gauss-newton
    for (int iter = 0; iter < iterations; iter++)
    {
        // init H, b, cost
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();
        cost = 0;

        // compute cost, J, H, b
        for (int i = 0; i < points_3d.size(); i++)
        {
            // 3d reporject to img2 camera
            Eigen::Vector3d pc = pose * points_3d[i];
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);

            // reprojection error
            Eigen::Vector2d e = points_2d[i] - proj;

            // cost
            cost += e.squaredNorm();

            // Jacobean matrix
            Eigen::Matrix<double, 2, 6> J;
            J << -fx * inv_z, 0, fx * pc[0] * inv_z2, fx * pc[0] * pc[1] * inv_z2, -fx - fx * pc[0] * pc[0] * inv_z2, fx * pc[1] * inv_z, 0,
                -fy * inv_z, fy * pc[1] * inv_z2, fy + fy * pc[1] * pc[1] * inv_z2, -fy * pc[0] * pc[1] * inv_z2, -fy * pc[0] * inv_z;

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        // solve H*x = b
        Vector6d dx;
        dx = H.ldlt().solve(b);

        // check NaN
        if (std::isnan(dx[0]))
        {
            std::cout << "result is nan!" << std::endl;
            break;
        }

        // check cost and iter
        if (iter > 0 && cost >= lastCost)
        {
            // cost increase, update is not good
            std::cout << "cost: " << cost << ", last cost: " << lastCost << std::endl;
            break;
        }

        // update estimation
        pose = Sophus::SE3d::exp(dx) * pose;
        lastCost = cost;

        std::cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << std::endl;

        // check dx convergence
        if (dx.norm() < 1e-6)
        {
            // converge
            break;
        }
    }

    std::cout << "pose by g-n: \n" << pose.matrix() << std::endl;
}

/**
 * @brief G2O vertex for BA, aka. Camera pose of img2
 *
 */
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override
    {
        _estimate = Sophus::SE3d();
    }

    // left multiplication on SE3
    // update SE3
    virtual void oplusImpl(const double *update) override
    {
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    virtual bool read(std::istream &in) override {}

    virtual bool write(std::ostream &out) const override {}
};

/**
 * @brief G2O edges for BA, aka. 3d pts projection in img2
 *
 */
class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos3d(pos), _K(K) {}

    // reprojection error
    virtual void computeError() override
    {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }

    // jacobean matrix
    virtual void linearizeOplus() override
    {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Eigen::Vector3d pos_cam = T * _pos3d;
        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double cx = _K(0, 2);
        double cy = _K(1, 2);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Z2 = Z * Z;
        _jacobianOplusXi << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z, 0, -fy / Z, fy * Y / (Z * Z),
            fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
    }

    virtual bool read(std::istream &in) override {}

    virtual bool write(std::ostream &out) const override {}

private:
    Eigen::Vector3d _pos3d;
    Eigen::Matrix3d _K;
};

/**
 * @brief BA by g2o
 *
 * @param points_3d, 3d pts from img1
 * @param points_2d, 2d pts from img2
 * @param K, camera intrinsic matrix
 * @param pose, cam pose between img1 and img2
 */
void bundleAdjustmentG2O(const VecVector3d &points_3d, const VecVector2d &points_2d, const cv::Mat &K, Sophus::SE3d &pose)
{
    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;            // pose is 6, landmark is 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;  // 线性求解器类型

    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;  // 图模型
    optimizer.setAlgorithm(solver);  // 设置求解器
    optimizer.setVerbose(true);      // 打开调试输出

    // vertex
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertex_pose);

    // K_eigen
    Eigen::Matrix3d K_eigen;
    K_eigen << K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2), K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
        K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

    // edges
    int index = 1;
    for (size_t i = 0; i < points_2d.size(); ++i)
    {
        auto p2d = points_2d[i];
        auto p3d = points_3d[i];
        EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);
        edge->setId(index);
        edge->setVertex(0, vertex_pose);
        edge->setMeasurement(p2d);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "optimization costs time: " << time_used.count() << " seconds." << std::endl;
    std::cout << "pose estimated by g2o =\n" << vertex_pose->estimate().matrix() << std::endl;
    pose = vertex_pose->estimate();
}
