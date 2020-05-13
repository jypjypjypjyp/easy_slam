#ifndef easy_slam_BACKEND_H
#define easy_slam_BACKEND_H

#include "easy_slam/common.h"
#include "easy_slam/frame.h"
#include "easy_slam/map.h"

namespace easy_slam
{
class Map;

class Backend
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    Backend();

    void SetCameras(Camera::Ptr left, Camera::Ptr right)
    {
        camera_left_ = left;
        camera_right_ = right;
    }

    void SetMap(std::shared_ptr<Map> map) { map_ = map; }

    void UpdateMap();

    void Stop();

private:
    void BackendLoop();

    void Optimize();

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr camera_left_ = nullptr, camera_right_ = nullptr;
};

} // namespace easy_slam

#endif // easy_slam_BACKEND_H