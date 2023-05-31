/**
 * @file optical_flow.cpp
 * @author your name (you@domain.com)
 * @brief optical flow example
 * @version 0.1
 * @date 2022-09-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>

// image data path
std::string file_1 = "../data/LK1.png"; // first image
std::string file_2 = "../data/LK2.png"; // second image

/**
 * @brief class OpticalFlowTracker
 * 
 */
class OpticalFlowTracker
{
public:
    // constructor
    OpticalFlowTracker(
        const cv::Mat &img1_,
        const cv::Mat &img2_,
        const std::vector<cv::KeyPoint> &kp1_,
        std::vector<cv::KeyPoint> &kp2_,
        std::vector<bool> &success_,
        bool inverse_ = true, bool has_initial_ = false) : img1(img1_), img2(img2_), kp1(kp1_), kp2(kp2_), success(success_), inverse(inverse_),
                                                           has_initial(has_initial_) {}
    // calculateOpticalFlow
    void calculateOpticalFlow(const cv::Range &range);

private:
    const cv::Mat &img1;                    // img1
    const cv::Mat &img2;                    // img2
    const std::vector<cv::KeyPoint> &kp1;   // kp1, no change
    std::vector<cv::KeyPoint> &kp2;         // kp2
    std::vector<bool> &success;             // success flag
    bool inverse = true;                    // use inverse formulation
    bool has_initial = false;               // has initial estimation?
};

/**
 * @brief single level optical flow
 * 
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in,out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 * @param [in] has_initial_guess, prefix=false
 */
void OpticalFlowSingleLevel(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const std::vector<cv::KeyPoint> &kp1,
    std::vector<cv::KeyPoint> &kp2,
    std::vector<bool> &success,
    bool inverse = false,
    bool has_initial_guess = false);

/**
 * @brief multi level optical flow, scale of pyramid is set to 2 by default
 *        the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const std::vector<cv::KeyPoint> &kp1,
    std::vector<cv::KeyPoint> &kp2,
    std::vector<bool> &success,
    bool inverse = false);

/**
 * @brief get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return the interpolated value of this pixel
 */
inline float GetPixelValue(const cv::Mat &img, float x, float y)
{
    // boundary check
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if (x >= img.cols - 1)
        x = img.cols - 2;
    if (y >= img.rows - 1)
        y = img.rows - 2;

    // (int)xx <= (float)x <= (int)x_a1
    // (int)yy <= (float)y <= (int)y_a1
    float xx = x - floor(x);
    float yy = y - floor(y);
    int x_a1 = std::min(img.cols - 1, int(x) + 1);
    int y_a1 = std::min(img.rows - 1, int(y) + 1);

    // Bi-Interpolation
    float pixel_value = (1 - xx) * (1 - yy) * img.at<uchar>(y, x) + 
        xx * (1 - yy) * img.at<uchar>(y, x_a1) + 
        (1 - xx) * yy * img.at<uchar>(y_a1, x) + 
        xx * yy * img.at<uchar>(y_a1, x_a1);

    return pixel_value;
}

/**
 * @brief main, optical flow example
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    // images, note they are CV_8UC1, not CV_8UC3
    cv::Mat img1 = cv::imread(file_1, 0);
    cv::Mat img2 = cv::imread(file_2, 0);

    // img1 key points, using GFTT here.
    // useHarrisDetector 决定使用 Harris 判定依据还是 Shi-Tomasi 判定依据
    std::vector<cv::KeyPoint> kp1;
    cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1);

    // now lets track these key points in the second image
    // first use single level LK in the validation picture
    std::vector<cv::KeyPoint> kp2_single;
    std::vector<bool> success_single;
    OpticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single);

    // then test multi-level LK
    std::vector<cv::KeyPoint> kp2_multi;
    std::vector<bool> success_multi;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi, success_multi, true);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "optical flow by gauss-newton: " << time_used.count() << std::endl;

    // use opencv's flow for validation
    std::vector<cv::Point2f> pt1, pt2;
    for (auto &kp : kp1)
    {
        pt1.push_back(kp.pt);
    }
    std::vector<uchar> status;
    std::vector<float> error;
    t1 = std::chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "optical flow by opencv: " << time_used.count() << std::endl;

    // plot the differences of those functions
    cv::Mat img2_single;
    cv::cvtColor(img2, img2_single, CV_GRAY2BGR);
    for (int i = 0; i < kp2_single.size(); i++)
    {
        if (success_single[i])
        {
            cv::circle(img2_single, kp2_single[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 250, 0));
        }
    }

    cv::Mat img2_multi;
    cv::cvtColor(img2, img2_multi, CV_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++)
    {
        if (success_multi[i])
        {
            cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 250, 0));
        }
    }

    cv::Mat img2_CV;
    cv::cvtColor(img2, img2_CV, CV_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++)
    {
        if (status[i])
        {
            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }

    cv::imshow("tracked single level", img2_single);
    cv::imshow("tracked multi level", img2_multi);
    cv::imshow("tracked by opencv", img2_CV);
    cv::waitKey(0);

    return 0;
}

/**
 * @brief single level optical flow
 * 
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in,out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const std::vector<cv::KeyPoint> &kp1,
    std::vector<cv::KeyPoint> &kp2,
    std::vector<bool> &success,
    bool inverse, bool has_initial)
{
    // kp2 size == kp1 size
    kp2.resize(kp1.size());

    // success flag size == kp1 size
    success.resize(kp1.size());

    // construct a new instance of OpticalFlowTracker
    OpticalFlowTracker tracker(img1, img2, kp1, kp2, success, inverse, has_initial);

    // 并行计算
    cv::parallel_for_(
        cv::Range(0, kp1.size()),
        std::bind(
            &OpticalFlowTracker::calculateOpticalFlow, 
            &tracker, 
            std::placeholders::_1)
        );
}

/**
 * @brief multi level optical flow, scale of pyramid is set to 2 by default
 *        the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const std::vector<cv::KeyPoint> &kp1,
    std::vector<cv::KeyPoint> &kp2,
    std::vector<bool> &success,
    bool inverse)
{
    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5; 
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::vector<cv::Mat> pyr1, pyr2; // image pyramids
    for (int i = 0; i < pyramids; i++)
    {
        if (i == 0)
        {
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        }
        else
        {
            // 新的图层为之前图层长宽尺寸的0.5
            cv::Mat img1_pyr, img2_pyr;
            cv::resize(pyr1[i - 1], img1_pyr,
                       cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
            cv::resize(pyr2[i - 1], img2_pyr,
                       cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
            pyr1.push_back(img1_pyr);
            pyr2.push_back(img2_pyr);
        }
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "build pyramid time: " << time_used.count() << std::endl;

    // coarse-to-fine LK tracking in pyramids
    // init coarse kp
    std::vector<cv::KeyPoint> kp1_pyr, kp2_pyr;
    for (auto &kp : kp1)
    {
        auto kp_top = kp;
        kp_top.pt *= scales[pyramids - 1];  // 0.125
        kp1_pyr.push_back(kp_top);
        kp2_pyr.push_back(kp_top);
    }

    // from coarse to fine
    for (int level = pyramids - 1; level >= 0; level--)
    {
        success.clear();
        t1 = std::chrono::steady_clock::now();
        OpticalFlowSingleLevel(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse, true);
        t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "track pyr " << level << " cost time: " << time_used.count() << std::endl;

        // recover kp from coarse kp
        if (level > 0)
        {
            // kp1
            for (auto &kp : kp1_pyr)
                kp.pt /= pyramid_scale;

            // kp2, coarse kp2 will be used as initialized kp2 in next fine pyramid layer
            for (auto &kp : kp2_pyr)
                kp.pt /= pyramid_scale;
        }
    }

    // save kp2
    for (auto &kp : kp2_pyr)
        kp2.push_back(kp);
}

/**
 * @brief calculateOpticalFlow using Gauss-Newton optimize method
 * 
 * @param range 
 */
void OpticalFlowTracker::calculateOpticalFlow(const cv::Range &range)
{
    // parameters
    int half_patch_size = 4;    // half_patch_size for kp patch window
    int iterations = 10;        // iterations for gauss-newton optimization

    // iterate this range of kp1 vec
    for (size_t i = range.start; i < range.end; i++)
    {
        // kp from kp1
        auto kp = kp1[i];

        // dx,dy need to be estimated
        double dx = 0, dy = 0; 

        // if kp2 has initial estimation, prefix=false
        if (has_initial)
        {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        // param for gauss newton
        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero(); // hessian
        Eigen::Vector2d b = Eigen::Vector2d::Zero(); // bias
        Eigen::Vector2d J;                           // jacobian
        for (int iter = 0; iter < iterations; iter++)
        {
            // check inverse formulation
            if (inverse == false)
            {
                H = Eigen::Matrix2d::Zero(); 
                b = Eigen::Vector2d::Zero();
            }
            else
            {
                // only reset b
                b = Eigen::Vector2d::Zero();
            }

            cost = 0;

            // compute cost and jacobian
            // cost from a patch window
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++)
                {
                    // e = (I(x, y) - I(x+dx, y+dy))
                    double error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) -
                                   GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy);
                    
                    // Jacobian
                    if (inverse == false)
                    {
                        // -(Ix, Iy), -(grad_x, grad_y)
                        // if not in inverse mode, update J every time
                        J = -1.0 * Eigen::Vector2d(
                                0.5 * (GetPixelValue(img2, kp.pt.x + dx + x + 1, kp.pt.y + dy + y) -
                                        GetPixelValue(img2, kp.pt.x + dx + x - 1, kp.pt.y + dy + y)),
                                0.5 * (GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y + 1) -
                                        GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y - 1)));
                    }
                    else if (iter == 0)
                    {
                        // in inverse mode, J keeps same for all iterations
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                        J = -1.0 * Eigen::Vector2d(
                                       0.5 * (GetPixelValue(img1, kp.pt.x + x + 1, kp.pt.y + y) -
                                              GetPixelValue(img1, kp.pt.x + x - 1, kp.pt.y + y)),
                                       0.5 * (GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y + 1) -
                                              GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y - 1)));
                    }
                    // compute H, b and set cost;
                    b += -error * J;
                    cost += error * error;
                    if (inverse == false || iter == 0)
                    {
                        // also update H if not in inverse mode
                        // or in inverse mode and iter == 0
                        H += J * J.transpose();
                    }
                }

            // compute update
            Eigen::Vector2d update = H.ldlt().solve(b);

            // check NAN
            if (std::isnan(update[0]))
            {
                // sometimes occurred when we have a black or white patch and H is irreversible
                std::cout << "update is nan" << std::endl;
                succ = false;
                break;
            }

            // check cost
            // if cost > last_cost, break the loop 
            if (iter > 0 && cost > lastCost)
            {
                break;
            }

            // update dx, dy, lastcost, succ_flag
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;

            // check update convergence
            if (update.norm() < 1e-2)
            {
                // converge
                break;
            }
        }

        // save succ flag
        success[i] = succ;

        // save kp2
        kp2[i].pt = kp.pt + cv::Point2f(dx, dy);
    }
}