#ifndef easy_slam_DATASET_H
#define easy_slam_DATASET_H
#include "easy_slam/camera.h"
#include "easy_slam/common.h"
#include "easy_slam/frame.h"

namespace easy_slam
{

class Dataset
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;
    Dataset(const std::string &dataset_path);

    bool Init();

    /// create and return the next frame containing the stereo images
    Frame::Ptr NextFrame();

    /// get camera by id
    Camera::Ptr GetCamera(int camera_id) const
    {
        return cameras_.at(camera_id);
    }

private:
    std::string dataset_path_;
    int current_image_index_ = 0;

    std::vector<Camera::Ptr> cameras_;
};
} // namespace easy_slam

#endif