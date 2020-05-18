#ifndef easy_slam_REPROJECTION_ERROR_H
#define easy_slam_REPROJECTION_ERROR_H

#include "easy_slam/camera.h"
#include "easy_slam/utility.h"
#include <ceres/ceres.h>

namespace easy_slam
{

class ReprojectionError : public ceres::SizedCostFunction<2, 7, 3>
{
public:
    ReprojectionError(Vec2 observation, Camera::Ptr camera)
        : observation_(observation), camera_(camera)
    {
        Eigen::LLT<Mat22> llt(Mat22::Identity().inverse());
        sqrt_information_ = llt.matrixU();
    }

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const
    {
        Eigen::Map<SE3 const> Tcw(parameters[0]);
        Eigen::Map<Vec3 const> point(parameters[1]);
        Eigen::Map<Vec2> residual(residuals);
        Vec3 P_c = camera_->world2camera(point, Tcw);

        residual = sqrt_information_ * (observation_ - camera_->camera2pixel(P_c));

        if (jacobians)
        {
            Eigen::Matrix<double, 2, 3> jaco_res_2_Pc;
            jaco_res_2_Pc << -camera_->fx_ / P_c(2), 0, camera_->fx_ * P_c(0) / (P_c(2) * P_c(2)),
                0, -camera_->fy_ / P_c(2), camera_->fy_ * P_c(1) / (P_c(2) * P_c(2));
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian(jacobians[0]);

                Eigen::Matrix<double, 3, 7> jaco_Pc_2_Pose;
                jaco_Pc_2_Pose.setZero();
                jaco_Pc_2_Pose.block<3, 3>(0, 0) = -Sophus::SO3d::hat(P_c);
                jaco_Pc_2_Pose.block<3, 3>(0, 4) = Eigen::Matrix3d::Identity();

                jacobian = sqrt_information_ * jaco_res_2_Pc * jaco_Pc_2_Pose;
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian(jacobians[1]);
                Eigen::Matrix3d R = camera_->pose().rotationMatrix() * Tcw.rotationMatrix();
                LOG(INFO) << jaco_res_2_Pc;
                LOG(INFO) << R;
                LOG(INFO) << sqrt_information_;
                jacobian = sqrt_information_ * jaco_res_2_Pc * R;
            }
        }

        return true;
    }

private:
    Vec2 observation_;
    Camera::Ptr camera_;
    Mat22 sqrt_information_;
};

} // namespace easy_slam

#endif // easy_slam_POSE_ONLY_REPROJECTION_ERROR_H
