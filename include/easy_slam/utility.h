#ifndef easy_slam_UTILITY_H
#define easy_slam_UTILITY_H

// utilities used in easy_slam
#include "easy_slam/common.h"
#include <algorithm>
#include <cmath>
#include <limits>
namespace easy_slam
{

/**
 * linear triangulation with SVD
 * @param poses     poses,
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 */
inline bool triangulation(const std::vector<SE3> &poses,
                          const std::vector<Vec3> points, Vec3 &pt_world)
{
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i)
    {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (pt_world[2] < 0)
    {
        return false;
    }
    return true;
}

// converters
inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }

inline Vec3 toVec3(double *p)
{
    return Vec3(p[0], p[1], p[2]);
}

inline double *toDouble(const Vec3 pose)
{
    double *array = new double[3];
    array[0] = pose.x();
    array[1] = pose.y();
    array[2] = pose.z();
    return array;
}

inline double *toDouble(const SE3 pose)
{
    Eigen::Quaterniond q = pose.unit_quaternion();
    Vec3 t = pose.translation();
    double *array = new double[7];
    array[0] = q.x();
    array[1] = q.y();
    array[2] = q.z();
    array[3] = q.w();
    array[4] = t.x();
    array[5] = t.y();
    array[6] = t.z();
    return array;
}

inline SE3 toSE3(double *array)
{
    Eigen::Quaterniond q(array[0], array[1], array[2], array[3]);
    Vec3 t(array[4], array[5], array[6]);
    return SE3(q, t);
}

inline Vec2 ReprojectionError(Vec2 observation, Mat33 K, SE3 camera_ext, SE3 pose, Vec3 point)
{
    Vec3 pos_pixel = K * (camera_ext * (pose * point));
    pos_pixel /= pos_pixel[2];
    return observation - pos_pixel.head<2>();
}

} // namespace easy_slam

#endif // easy_slam_UTILITY_H
