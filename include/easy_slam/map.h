
#ifndef MAP_H
#define MAP_H

#include "easy_slam/common.h"
#include "easy_slam/frame.h"
#include "easy_slam/mappoint.h"

namespace easy_slam
{

class Map
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;
    typedef std::unordered_map<unsigned long, double *> ParamsType;

    Map() {}

    void InsertKeyFrame(Frame::Ptr frame);
    void InsertMapPoint(MapPoint::Ptr map_point);

    LandmarksType GetAllMapPoints()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }
    KeyframesType GetAllKeyFrames()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    LandmarksType GetActiveMapPoints()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    KeyframesType GetActiveKeyFrames()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    ParamsType GetPoseParams();

    ParamsType GetPointParams();

    void UpdateMap();

    void CleanMap();

private:
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;       
    LandmarksType active_landmarks_;
    KeyframesType keyframes_;       
    KeyframesType active_keyframes_;

    std::unordered_map<unsigned long, double *> para_Pose;
    std::unordered_map<unsigned long, double *> para_Point;

    Frame::Ptr current_frame_ = nullptr;

    // settings
    int num_active_keyframes_ = 7;
};
} // namespace easy_slam

#endif // MAP_H
