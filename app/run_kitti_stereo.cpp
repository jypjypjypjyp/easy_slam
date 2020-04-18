//
// Created by gaoxiang on 19-5-4.
//

#include <gflags/gflags.h>
#include "easy_slam/visual_odometry.h"

DEFINE_string(config_file, "./config/default.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    easy_slam::VisualOdometry::Ptr vo(
        new easy_slam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}
