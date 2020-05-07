#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "easy_slam/utility.h"

namespace easy_slam
{

class ReprojectionFactor
{
public:
    ReprojectionFactor(Vec2 observation, Mat33 K, SE3 ext) : _observation(observation),
                                                             _ext(ext),
                                                             _K(K) {}

    bool operator()(const double *const pose, const double *const point, double *residuals) const
    {
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
        Vec3 t(pose[0], pose[1], pose[2]);
        SE3 se3 = SE3(q, t);
        Vec3 p = Vec3(point[0], point[1], point[2]);
        Vec2 error = ReprojectionError(_observation, _K, _ext, se3, p);
        residuals[0] = error[0];
        residuals[1] = error[1];
        return true;
    }

    static ceres::CostFunction *Build(const Vec2 observation, const Mat33 K, const SE3 ext)
    {
        return (new ceres::NumericDiffCostFunction<ReprojectionFactor, ceres::CENTRAL, 2, 7, 3>(
            new ReprojectionFactor(observation, K, ext)));
    }

private:
    Mat33 _K;
    Vec2 _observation;
    SE3 _ext;
};

} // namespace easy_slam