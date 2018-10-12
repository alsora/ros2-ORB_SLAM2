# ORBSLAM2 ROS2 node

This node implements a simple `Image` subscriber ROS2 node, which wraps the ORBSLAM2 library.


### Requirements

 - ROS2 Bouncy
 - Pangolin
 - ORB_SLAM2

### Setup

Build Pangolin

Build ORB_SLAM2

Install ORB_SLAM2

Build this package

    $ colcon build
    $ source install/setup.sh
    $ export LD_LIBRARY_PATH=~/orbslam/Pangolin/build/src/:$LD_LIBRARY_PATH
    $ ros2 run orbslam mono PATH_TO_FEATURES_VOCAB PATH_TO_CONFIG_FILE

