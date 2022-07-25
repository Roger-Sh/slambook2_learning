#include <iostream>
#include <opencv2/opencv.hpp>
// #include "extra.h" // used in opencv2

/**
 * @brief find match feature
 * 
 * @param img_1 
 * @param img_2 
 * @param keypoints_1 
 * @param keypoints_2 
 * @param matches 
 */
void find_feature_matches(
    const cv::Mat &img_1, const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches);

/**
 * @brief estimate pose based on 2d-2d
 * 
 * @param keypoints_1 
 * @param keypoints_2 
 * @param matches 
 * @param R 
 * @param t 
 */
void pose_estimation_2d2d(
    const std::vector<cv::KeyPoint> &keypoints_1,
    const std::vector<cv::KeyPoint> &keypoints_2,
    const std::vector<cv::DMatch> &matches,
    cv::Mat &R, cv::Mat &t);

/**
 * @brief triangulation
 * 
 * @param keypoint_1 
 * @param keypoint_2 
 * @param matches 
 * @param R 
 * @param t 
 * @param points 
 */
void triangulation(
    const std::vector<cv::KeyPoint> &keypoint_1,
    const std::vector<cv::KeyPoint> &keypoint_2,
    const std::vector<cv::DMatch> &matches,
    const cv::Mat &R, const cv::Mat &t,
    std::vector<cv::Point3d> &points);

/**
 * @brief Get the color object, for drawing
 * inline for small func, which will be called many times
 * 
 * @param depth 
 * @return cv::Scalar 
 */
inline cv::Scalar get_color(float depth)
{
    float up_th = 50, low_th = 10, th_range = up_th - low_th;
    if (depth > up_th)
        depth = up_th;
    if (depth < low_th)
        depth = low_th;
    return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}

/**
 * @brief 像素坐标转相机归一化坐标
 * 
 * @param p 
 * @param K 
 * @return cv::Point2f 
 */
cv::Point2f pixel2cam(const cv::Point2d &p, const cv::Mat &K);


/**
 * @brief main, test triangulation
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    // check args: img1, img2
    if (argc != 3)
    {
        std::cout << "usage: triangulation img1 img2" << std::endl;
        return 1;
    }
    // read img
    cv::Mat img_1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);

    // find ORB feature matches
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    std::cout << "一共找到了" << matches.size() << "组匹配点" << std::endl;

    // estimate pose 2d-2d
    cv::Mat R, t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

    // triangulation
    std::vector<cv::Point3d> points;
    triangulation(keypoints_1, keypoints_2, matches, R, t, points);

    // reprojection for validation of triangulation
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    cv::Mat img1_plot = img_1.clone();
    cv::Mat img2_plot = img_2.clone();
    for (int i = 0; i < matches.size(); i++)
    {
        // img1 draw original keypoints with depth
        float depth1 = points[i].z;
        std::cout << "depth: " << depth1 << std::endl;
        cv::Point2d pt1_cam = pixel2cam(keypoints_1[matches[i].queryIdx].pt, K);
        cv::circle(img1_plot, keypoints_1[matches[i].queryIdx].pt, 2, get_color(depth1), 2);

        // img1 draw reprojected points
        int img1_u = (K.at<double>(0, 0)*points[i].x + K.at<double>(0, 2)*points[i].z)/points[i].z;
        int img1_v = (K.at<double>(1, 1)*points[i].y + K.at<double>(1, 2)*points[i].z)/points[i].z;
        cv::circle(img1_plot, cv::Point(img1_u, img1_v), 10, get_color(depth1), 2);

        // img2, draw original keypoints with depth
        cv::Mat pt2_trans = R * (cv::Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
        float depth2 = pt2_trans.at<double>(2, 0);
        cv::circle(img2_plot, keypoints_2[matches[i].trainIdx].pt, 2, get_color(depth2), 2);

        // img2 draw reprojected points
        int img2_u = (K.at<double>(0, 0)*pt2_trans.at<double>(0, 0) + K.at<double>(0, 2)*pt2_trans.at<double>(2, 0))/pt2_trans.at<double>(2, 0);
        int img2_v = (K.at<double>(1, 1)*pt2_trans.at<double>(1, 0) + K.at<double>(1, 2)*pt2_trans.at<double>(2, 0))/pt2_trans.at<double>(2, 0);
        cv::circle(img2_plot, cv::Point(img2_u, img2_v), 10, get_color(depth2), 2);

        
    }
    cv::imshow("img 1", img1_plot);
    cv::imshow("img 2", img2_plot);
    cv::waitKey();

    return 0;
}

/**
 * @brief find match feature
 * 
 * @param img_1 
 * @param img_2 
 * @param keypoints_1 
 * @param keypoints_2 
 * @param matches 
 */
void find_feature_matches(const cv::Mat &img_1, const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches)
{
    // init descriptor
    cv::Mat descriptors_1, descriptors_2;

    // init ORB detector, extractor, matcher
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" )
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
    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    double min_dist = 10000, max_dist = 0;
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

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= std::max(2 * min_dist, 30.0))
        {
            matches.push_back(match[i]);
        }
    }
}

/**
 * @brief estimate pose based on 2d-2d
 * 
 * @param keypoints_1 
 * @param keypoints_2 
 * @param matches 
 * @param R 
 * @param t 
 */
void pose_estimation_2d2d(
    const std::vector<cv::KeyPoint> &keypoints_1,
    const std::vector<cv::KeyPoint> &keypoints_2,
    const std::vector<cv::DMatch> &matches,
    cv::Mat &R, cv::Mat &t)
{
    // 相机内参,TUM Freiburg2
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    //-- 把匹配点转换为vector<Point2f>的形式
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    for (int i = 0; i < (int)matches.size(); i++)
    {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    //-- 计算本质矩阵
    cv::Point2d principal_point(325.1, 249.7); //相机主点, TUM dataset标定值
    int focal_length = 521;                //相机焦距, TUM dataset标定值
    cv::Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
}

/**
 * @brief triangulation
 * 
 * @param keypoint_1 
 * @param keypoint_2 
 * @param matches 
 * @param R 
 * @param t 
 * @param points 
 */
void triangulation(
    const std::vector<cv::KeyPoint> &keypoint_1,
    const std::vector<cv::KeyPoint> &keypoint_2,
    const std::vector<cv::DMatch> &matches,
    const cv::Mat &R, const cv::Mat &t,
    std::vector<cv::Point3d> &points)
{
    // 相机之间的转换矩阵
    cv::Mat T1 = (cv::Mat_<float>(3, 4) << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0);
    cv::Mat T2 = (cv::Mat_<float>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
              R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
              R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

    // 相机内参矩阵
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    // 将像素坐标转换至相机坐标
    std::vector<cv::Point2f> pts_1, pts_2;
    for (cv::DMatch m : matches)
    {
        pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
        pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
    }

    // 特征点三角定位
    cv::Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    // 转换成非齐次坐标
    for (int i = 0; i < pts_4d.cols; i++)
    {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // 归一化
        cv::Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0));
        points.push_back(p);
    }
}

/**
 * @brief 像素坐标转相机归一化坐标
 * 
 * @param p 
 * @param K 
 * @return cv::Point2f 
 */
cv::Point2f pixel2cam(const cv::Point2d &p, const cv::Mat &K)
{
    return cv::Point2f(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}
