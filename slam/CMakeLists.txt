cmake_minimum_required(VERSION 3.16)
project(slam_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(stella_vslam REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

add_executable(run_slam run_slam.cc)
target_link_libraries(run_slam PRIVATE
    stella_vslam::stella_vslam
    opencv_core
    opencv_imgcodecs
)

install(TARGETS run_slam DESTINATION bin)
