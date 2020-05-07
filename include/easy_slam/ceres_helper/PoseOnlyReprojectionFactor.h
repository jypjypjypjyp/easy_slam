#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "easy_slam/utility.h"

namespace easy_slam
{

class PoseOnlyReprojectionFactor
{
public:
    PoseOnlyReprojectionFactor(Vec2 observation, Mat33 K, SE3 ext, Vec3 point) : _observation(observation),
                                                                                 _ext(ext),
                                                                                 _K(K),
                                                                                 _point(point) {}

    bool operator()(const double *const q, const double *const t, double *residuals) const
    {
        Eigen::Quaterniond _q(q);
        Vec3 _t(t);
        SE3 se3 = SE3(_q, _t);
        Vec2 error = ReprojectionError(_observation, _K, _ext, se3, _point);
        residuals[0] = error[0];
        residuals[1] = error[1];
        return true;
    }

    static ceres::CostFunction *Build(const Vec2 observation, const Mat33 K, const SE3 ext, Vec3 point)
    {
        return (new ceres::NumericDiffCostFunction<PoseOnlyReprojectionFactor, ceres::CENTRAL, 2, 4, 3>(
            new PoseOnlyReprojectionFactor(observation, K, ext, point)));
    }

private:
    Mat33 _K;
    Vec2 _observation;
    SE3 _ext;
    Vec3 _point;
};

} // namespace easy_slam