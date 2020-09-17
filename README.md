# ros2-ORB_SLAM2
ROS2 node wrapping the ORB_SLAM2 library

If you want to integrate ORB_SLAM2 inside your ROS2 system, consider trying [this](https://github.com/alsora/ORB_SLAM2) fork of ORB_SLAM2 library which drops Pangolin dependency and streams all SLAM data through ROS2 topics.

### Requirements

 - [ROS2 Foxy](https://github.com/ros2/ros2/wiki/Installation)
 - [Pangolin](https://github.com/stevenlovegrove/Pangolin)
 - [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
 - [OpenCV3](https://docs.opencv.org/3.0-beta/doc/tutorials/introduction/linux_install/linux_install.html)
 - [vision_opencv](https://github.com/ros-perception/vision_opencv/tree/ros2)
 - [message_filters](https://github.com/ros2/message_filters)

Note: The `vision_opencv` package requires OpenCV3. Make sure to build ORB_SLAM2 with the same OpenCV version otherwise strange run errors could appear.

The `message_filters` package is not required if you want to use only the Monocular SLAM node. 


### Build

This repository contains a Dockerfile providing an Ubuntu environment with this package and all its dependencies already installed.
In order to use it:

    $ cd docker
    $ bash build.sh
    $ bash run.sh

Otherwise you can build the package on your system.
If you built ORB_SLAM2 following the instructions provided in its repository, you will have to tell CMake where to find it by exporting an environment variable that points to the cloned repository (as the library and include files will be in there).

    $ export ORB_SLAM2_ROOT_DIR=/path/to/ORB_SLAM2

Then you can build this package

    $ mkdir -p ws/src
    $ cd ws/src
    $ git clone https://github.com/alsora/ros2-ORB_SLAM2
    $ cd ..
    $ colcon build

### Usage

First source the workspace

    $ source ws/install/setup.sh

Then add to the LD_LIBRARY_PATH the location of ORB_SLAM2 library and its dependencies

    $ export LD_LIBRARY_PATH=:~/ORB_SLAM2/Thirdparty/DBoW2/lib:~/ORB_SLAM2/Thirdparty/g2o/lib:~/ORB_SLAM2/lib:$LD_LIBRARY_PATH

Run the monocular SLAM node

    $ ros2 run ros2_orbslam mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE

You can find the vocabulary file in the ORB_SLAM2 repository (e.g. `ORB_SLAM2/Vocabulary/ORBvoc.txt`), while the config file can be found within this repo (e.g. `ros2-ORB_SLAM2/src/monocular/TUM1.yaml` for monocular SLAM).

This node subscribes to the ROS2 topic `camera` and waits for Image messages.
For example you can stream frames from your laptop webcam using:

    $ ros2 run image_tools cam2image -t camera

The other nodes can be run in a very similar way.

You can run the `rgbd` node by using 

    $ ros2 run ros2_orbslam rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE

You can run the `stereo` node by using 

    $ ros2 run ros2_orbslam stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
