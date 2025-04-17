#!/bin/bash
sudo apt install -y libglm-dev libglfw3-dev libpng-dev libjpeg-dev libeigen3-dev libboost-filesystem-dev libboost-program-options-dev
git clone https://github.com/koide3/iridescence.git
cd iridescence
git checkout 085322e0c949f75b67d24d361784e85ad7f197ab
git submodule update --init --recursive
mkdir -p build
cd build
cmake \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    ..
make -j$(($(nproc) / 2))
sudo make install

rosdep update
sudo apt update
mkdir -p ~/lib
cd ~/lib
git clone --recursive --depth 1 https://github.com/stella-cv/stella_vslam.git
rosdep install -y -i --from-paths ~/lib
cd ~/lib/stella_vslam
mkdir -p ~/lib/stella_vslam/build
cd ~/lib/stella_vslam/build
source /opt/ros/${ROS_DISTRO}/setup.bash
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j
sudo make install

cd ~/lib
git clone --recursive https://github.com/stella-cv/iridescence_viewer.git
mkdir -p iridescence_viewer/build
cd iridescence_viewer/build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j
sudo make install

# had to make some changes to the original repo to run it in jazzy
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive --depth 1 https://github.com/oscarpoudel/stella_vslam_ros2_Jazzy.git
cd ~/ros2_ws
rosdep install -y -i --from-paths ~/ros2_ws/src --skip-keys=stella_vslam

colcon build --symlink-install