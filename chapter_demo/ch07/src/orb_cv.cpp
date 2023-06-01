#include <chrono>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

std::string img1_path = "../../../chapter_demo/ch07/image/1.png";
std::string img2_path = "../../../chapter_demo/ch07/image/2.png";

/**
 * @brief using CV ORB extractor
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    // check image path
    // if (argc != 3)
    // {
    //     std::cout << "usage: orb_cv img1_path img2_path" << std::endl;
    //     return 1;
    // }

    //-- 读取图像
    cv::Mat img_1 = cv::imread(img1_path, CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(img2_path, CV_LOAD_IMAGE_COLOR);
    assert(img_1.data != nullptr && img_2.data != nullptr);

    //-- 初始化
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;                                            // 关键点
    cv::Mat descriptors_1, descriptors_2;                                                          // 描述子
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();                                     // 特征检测器
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();                               // 描述子提取器
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");  // 描述子匹配器

    //-- 第一步:检测 Oriented FAST 角点位置
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "extract ORB cost = " << time_used.count() << " seconds. " << std::endl;

    // draw ORB features
    cv::Mat outimg1, outimg2;
    drawKeypoints(img_1, keypoints_1, outimg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    drawKeypoints(img_2, keypoints_2, outimg2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("ORB features img1", outimg1);
    cv::imshow("ORB features img2", outimg2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> matches;
    t1 = std::chrono::steady_clock::now();
    matcher->match(descriptors_1, descriptors_2, matches);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "match ORB cost = " << time_used.count() << " seconds. " << std::endl;

    //-- 第四步:匹配点对筛选
    // 计算最小距离和最大距离
    auto min_max =
        minmax_element(matches.begin(), matches.end(), [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;
    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (matches[i].distance <= std::max(2 * min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }
    }

    //-- 第五步:绘制匹配结果
    cv::Mat img_match;
    cv::Mat img_goodmatch;
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
    cv::imshow("all matches", img_match);
    cv::imshow("good matches", img_goodmatch);
    cv::waitKey(0);

    return 0;
}
