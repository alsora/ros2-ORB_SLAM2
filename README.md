# ros2-ORB_SLAM2
ROS2 node wrapping the ORB_SLAM2 library


### Requirements

 - [ROS2 Bouncy](https://github.com/ros2/ros2/wiki/Installation)
 - [Pangolin](https://github.com/stevenlovegrove/Pangolin)
 - [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
 - [OpenCV3](https://docs.opencv.org/3.0-beta/doc/tutorials/introduction/linux_install/linux_install.html)
 - [vision_opencv](https://github.com/ros-perception/vision_opencv/tree/ros2)
 - [message_filters](https://github.com/ros2/message_filters)

Note: The `vision_opencv` package requires OpenCV3. Make sure to build ORB_SLAM2 with the same OpenCV version otherwise strange run errors could appear.

The `message_filters` package is not required if you want to use only the Monocular SLAM node. 


This repository contains a Dockerfile providing an Ubuntu environment with all the dependences already installed.
In order to use it

    $ cd docker_
    $ bash build.sh
    $ bash run.sh

**Note** The `CMakeLists.txt` file contains hardcoded the path to the ORB_SLAM2 source directory. This is used to retrieve header files and shared libraries since these are not installed by ORB_SLAM2.

If you are not using the Dockerfile, change the following line with the actual ORB_SLAM2 path.

    set(ORB_SLAM2_DIR /root/ORB_SLAM2)


### Usage

Run the monocular SLAM node

    $ ros2 run orbslam mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE

This node subscribes to the ROS2 topic `camera` and waits for Image messages.

For example you can stream frames from your laptop webcam using:

    $ ros2 run image_tools cam2image -t camera


Similarly you can run the `rgbd` node by using 

    $ ros2 run orbslam rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE

