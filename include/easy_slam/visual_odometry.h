
#ifndef easy_slam_VISUAL_ODOMETRY_H
#define easy_slam_VISUAL_ODOMETRY_H

#include "easy_slam/backend.h"
#include "easy_slam/common.h"
#include "easy_slam/dataset.h"
#include "easy_slam/frontend.h"
#include "easy_slam/viewer.h"

namespace easy_slam
{

class VisualOdometry
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    /// conclassor with config file
    VisualOdometry(std::string &config_path);

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init();

    /**
     * start vo in the dataset
     */
    void Run();

    /**
     * Make a step forward in dataset
     */
    bool Step();

    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

private:
    bool inited_ = false;
    std::string config_file_path_;

    Frontend::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

    // dataset
    Dataset::Ptr dataset_ = nullptr;
};
} // namespace easy_slam

#endif // easy_slam_VISUAL_ODOMETRY_H
