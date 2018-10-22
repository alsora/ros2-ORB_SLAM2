# ros2-ORB_SLAM2
ROS2 node wrapping the ORB_SLAM2 library

**NOTE: This is a developer branch which works with my modified version of ORB_SLAM2.**

The new ORB_SLAM2 library doesn't use `Pangolin` for visualization, but publishes all data through ROS2 topcis.


### Requirements

 - [ROS2 Bouncy](https://github.com/ros2/ros2/wiki/Installation)
 - [Developer ORB_SLAM2](https://github.com/alsora/ORB_SLAM2)
 - [OpenCV3](https://docs.opencv.org/3.0-beta/doc/tutorials/introduction/linux_install/linux_install.html)
 - [vision_opencv](https://github.com/ros-perception/vision_opencv/tree/ros2)
 - [message_filters](https://github.com/ros2/message_filters)

Note: The `vision_opencv` package requires OpenCV3. Make sure to build ORB_SLAM2 with the same OpenCV version otherwise strange run errors could appear.

The `message_filters` package is not required if you want to use only the Monocular SLAM node. 


[Here](https://github.com/alsora/ORB_SLAM2/tree/master/docker) you can find a Dockerfile providing an Ubuntu environment with all the dependences already installed.
In order to use it

    $ cd docker
    $ bash build.sh
    $ bash run.sh

### Usage

Run the monocular SLAM node

    $ ros2 run orbslam mono ~/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/ORB_SLAM2/ros2-ORB_SLAM2/src/monocular/TUM1.yaml

These paths are valid in the Docker environment, otherwise use your own paths.
The `ORB_SLAM2` library contains the aforementioned vocabulary file and several configuration files.

This node subscribes to the ROS2 topic `camera` and waits for Image messages.

For example you can stream frames from your laptop webcam using:

    $ ros2 run image_tools cam2image -t camera



