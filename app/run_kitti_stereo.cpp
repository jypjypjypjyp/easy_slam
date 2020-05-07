#include "easy_slam/visual_odometry.h"

int main(int argc, char **argv) {
    std::string config_file = "./config/default.yaml";
    easy_slam::VisualOdometry::Ptr vo(
        new easy_slam::VisualOdometry(config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}
