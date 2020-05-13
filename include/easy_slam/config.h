
#ifndef easy_slam_CONFIG_H
#define easy_slam_CONFIG_H

#include "easy_slam/common.h"

namespace easy_slam
{

class Config
{
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {} // private conclassor makes a singleton
public:
    ~Config(); // close the file when deconclassing

    // set a new config file
    static bool SetParameterFile(const std::string &filename);

    // access the parameter values
    template <typename T>
    static T Get(const std::string &key)
    {
        T result;
        Config::config_->file_[key] >> result;
        return result;
    }
};
} // namespace easy_slam

#endif // easy_slam_CONFIG_H
