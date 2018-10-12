# ros2-ORB_SLAM2
ROS2 node wrapping the ORB_SLAM2 library


### Requirements

 - [ROS2 Bouncy](https://github.com/ros2/ros2/wiki/Installation)
 - [Pangolin](https://github.com/stevenlovegrove/Pangolin)
 - [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
 - [VisionOpenCV](https://github.com/ros-perception/vision_opencv/tree/ros2)
 - [OpenCV3](https://docs.opencv.org/3.0-beta/doc/tutorials/introduction/linux_install/linux_install.html)

Note: The `VisionOpenCV` package requires OpenCV3. Be aware that a source of possible build/run errors is building ORB_SLAM2 with a certain OpenCV version and then running this node using a different one.


This repository contains a Dockerfile providing an Ubuntu environment with all the dependences already installed.
In order to use it

    $ cd docker
    $ bash build.sh
    $ bash run.sh

### Usage

Run the monocular SLAM node

    $ ros2 run orbslam mono ~/orbslam/ORB_SLAM2/Vocabulary/ORBvoc.txt.tar.gz ~/orbslam/ORB_SLAM2/Examples/Monocular/TUM1.yaml

These paths are valid in the Docker environment, otherwise use your own paths.
The `ORB_SLAM2` library contains the aforementioned vocabulary file and several configuration files.

This node subscribes to the ROS2 topic `camera` and waits for Image messages.

For example you can stream frames from your laptop webcam using:

    $ ros2 run image_tools cam2image -t camera



