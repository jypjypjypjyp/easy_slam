#include "easy_slam/backend.h"
#include "easy_slam/utility.h"
#include "easy_slam/feature.h"
#include "easy_slam/ceres_helper/ReprojectionFactor.h"
#include "easy_slam/map.h"
#include "easy_slam/mappoint.h"

namespace easy_slam
{

Backend::Backend()
{
    backend_running_.store(true);
    backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
}

void Backend::UpdateMap()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void Backend::Stop()
{
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();
}

void Backend::BackendLoop()
{
    while (backend_running_.load())
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);

        /// 后端仅优化激活的Frames和Landmarks
        Optimize();
    }
}

//TODO: update with ceres
void Backend::Optimize()
{
    Map::ParamsType para_kfs = map_->GetPoseParams();
    Map::ParamsType para_landmarks = map_->GetPointParams();
    Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
    Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();

    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

    // K & left_ext & right_ext
    Mat33 K = camera_left_->K();
    SE3 left_ext = camera_left_->pose();
    SE3 right_ext = camera_right_->pose();

    ceres::LocalParameterization *local_parameterization = new ceres::QuaternionParameterization();
    for (auto &para : para_kfs)
    {
        problem.AddParameterBlock(para.second, 4, local_parameterization);
        problem.AddParameterBlock(para.second + 4, 3);
    }

    for (auto &landmark : active_landmarks)
    {
        if (landmark.second->is_outlier_)
            continue;
        double *para = para_landmarks[landmark.first];
        problem.AddParameterBlock(para, 3);
        auto observations = landmark.second->GetObs();
        for (auto &obs : observations)
        {
            if (obs.lock() == nullptr)
                continue;
            auto feature = obs.lock();
            if (feature->is_outlier_ || feature->frame_.lock() == nullptr)
                continue;
            auto frame = feature->frame_.lock();
            auto iter = active_kfs.find(frame->keyframe_id_);
            if (iter == active_kfs.end())
                continue;
            auto keyframe = *iter;

            ceres::CostFunction *cost_function;
            if (feature->is_on_left_image_)
            {
                cost_function = ReprojectionFactor::Build(toVec2(feature->position_.pt), K, left_ext);
            }
            else
            {
                cost_function = ReprojectionFactor::Build(toVec2(feature->position_.pt), K, right_ext);
            }
            problem.AddResidualBlock(cost_function, loss_function, para_kfs[keyframe.first], para_kfs[keyframe.first] + 4, para);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 5;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // reject outliers
    double error[2] = {0};
    for (auto &landmark : active_landmarks)
    {
        if (landmark.second->is_outlier_)
            continue;
        auto observations = landmark.second->GetObs();
        for (auto &obs : observations)
        {
            if (obs.lock() == nullptr)
                continue;
            auto feat = obs.lock();
            if (feat->is_outlier_ || feat->frame_.lock() == nullptr)
                continue;
            auto frame = feat->frame_.lock();
            auto iter = active_kfs.find(frame->keyframe_id_);
            if (iter == active_kfs.end())
                continue;
            auto keyframe = *iter;

            if (feat->is_on_left_image_)
            {
                (ReprojectionFactor(toVec2(feat->position_.pt), K, left_ext))(para_kfs[keyframe.first], para_landmarks[landmark.first], error);
            }
            else
            {
                (ReprojectionFactor(toVec2(feat->position_.pt), K, right_ext))(para_kfs[keyframe.first], para_landmarks[landmark.first], error);
            }
            if (error[0] > 3 || error[1] > 3)
            {
                landmark.second->RemoveObservation(feat);
            }
        }
    }

    map_->UpdateMap();
}

} // namespace easy_slam