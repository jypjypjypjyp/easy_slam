add_library(easy_slam SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp)

target_link_libraries(easy_slam
        ${THIRD_PARTY_LIBS})
target_compile_features(easy_slam PRIVATE cxx_std_17)