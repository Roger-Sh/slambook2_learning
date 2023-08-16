#include <pangolin/pangolin.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <sophus/se3.hpp>

// trajectory file path
std::string groundtruth_file = "../../../../chapter_demo/ch04/example/groundtruth.txt";
std::string estimated_file = "../../../../chapter_demo/ch04/example/estimated.txt";

// TrajectoryType
typedef std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

/**
 * @brief draw trajectory
 *
 * @param gt: ground truth trajectory
 * @param esti: estimated trajectory
 */
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

/**
 * @brief read trajectory
 *
 * @param path: trajectory path file
 * @return TrajectoryType
 */
TrajectoryType ReadTrajectory(const std::string &path);

/**
 * @brief 本程序演示了如何通过 Sophus 计算真值轨迹与预测轨迹之间的误差
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    // read trajectory
    TrajectoryType groundtruth = ReadTrajectory(groundtruth_file);
    TrajectoryType estimated = ReadTrajectory(estimated_file);
    assert(!groundtruth.empty() && !estimated.empty());
    assert(groundtruth.size() == estimated.size());

    // compute RMSE 均方根误差 （即绝对轨迹误差 ATE）
    double rmse = 0;
    for (size_t i = 0; i < estimated.size(); i++)
    {
        Sophus::SE3d p1 = estimated[i], p2 = groundtruth[i];
        double error = (p2.inverse() * p1).log().norm();
        rmse += error * error;
    }
    rmse = rmse / double(estimated.size());
    rmse = sqrt(rmse);
    std::cout << "RMSE = " << rmse << std::endl;

    DrawTrajectory(groundtruth, estimated);
    return 0;
}

/**
 * @brief read trajectory from file
 *
 * @param path
 * @return TrajectoryType
 */
TrajectoryType ReadTrajectory(const std::string &path)
{
    std::ifstream fin(path);
    TrajectoryType trajectory;
    if (!fin)
    {
        std::cerr << "trajectory " << path << " not found." << std::endl;
        return trajectory;
    }

    while (!fin.eof())
    {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3d p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        trajectory.push_back(p1);
    }
    return trajectory;
}

/**
 * @brief draw trajectory using pangolin
 *
 * @param gt
 * @param esti
 */
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti)
{
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
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

    // draw trajectory
    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);

        // draw groundtruth trajectory
        for (size_t i = 0; i < gt.size() - 1; i++)
        {
            // blue for ground truth
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto p1 = gt[i], p2 = gt[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        // draw estimated trajectory
        for (size_t i = 0; i < esti.size() - 1; i++)
        {
            // red for estimated
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = esti[i], p2 = esti[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);  // sleep 5 ms
    }
}
