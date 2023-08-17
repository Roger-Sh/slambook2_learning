#include <pangolin/pangolin.h>

#include <boost/format.hpp>  // for formating strings
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

// 轨迹类型
typedef std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// 在pangolin中画图，已写好，无需调整
void showPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);

/**
 * @brief 本程序通过将5副彩色图对应的深度图结合他们的pose，计算并拼接点云，合成一个场景点云
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    // init
    std::vector<cv::Mat> colorImgs, depthImgs;  // 彩色图和深度图
    TrajectoryType poses;                       // 相机位姿

    // check file
    std::ifstream fin("../../../../chapter_demo/ch05/rgbd/pose.txt");
    if (!fin)
    {
        std::cerr << "请在有pose.txt的目录下运行此程序" << std::endl;
        return 1;
    }

    // read image and pose
    for (int i = 0; i < 5; i++)
    {
        // read image
        boost::format fmt("../../../../chapter_demo/ch05/rgbd/%s/%d.%s");  // 图像文件格式
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(), -1));  // 使用-1读取原始图像

        // read pose x y z q1 q2 q3 q4
        double data[7] = {0};
        for (auto &d : data)
        {
            fin >> d;
        }

        // create SE3 from pose
        Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]), Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(pose);
    }

    // 计算点云并拼接
    // 相机内参
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    pointcloud.reserve(1000000);

    // 遍历图片
    for (int i = 0; i < 5; i++)
    {
        std::cout << "转换图像中: " << i + 1 << std::endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Sophus::SE3d T = poses[i];

        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++)
            {
                // 深度值
                unsigned int d = depth.ptr<unsigned short>(v)[u];

                // 为0表示没有测量到
                if (d == 0)
                {
                    continue;
                }

                // 计算相机坐标
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;

                // 计算世界坐标
                Eigen::Vector3d pointWorld = T * point;

                // 获取颜色信息
                Vector6d p;
                p.head<3>() = pointWorld;
                p[5] = color.data[v * color.step + u * color.channels()];      // blue
                p[4] = color.data[v * color.step + u * color.channels() + 1];  // green
                p[3] = color.data[v * color.step + u * color.channels() + 2];  // red

                // 加入点云
                pointcloud.push_back(p);
            }
    }

    std::cout << "点云共有" << pointcloud.size() << "个点." << std::endl;
    showPointCloud(pointcloud);
    return 0;
}

/**
 * @brief show pointcloud using pangolin
 *
 * @param pointcloud
 */
void showPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud)
{
    // check empty
    if (pointcloud.empty())
    {
        std::cerr << "Point cloud is empty!" << std::endl;
        return;
    }

    // pangolin window
    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // pangolin s_cam
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    // pangolin d_cam
    pangolin::View &d_cam =
        pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f).SetHandler(new pangolin::Handler3D(s_cam));

    // draw point cloud
    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glPointSize(2);
        glBegin(GL_POINTS);

        // draw point
        for (auto &p : pointcloud)
        {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }

        glEnd();
        pangolin::FinishFrame();
        usleep(5000);  // sleep 5 ms
    }
    return;
}
