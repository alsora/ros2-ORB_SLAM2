#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"

#include"System.h"

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify" << std::endl;        
        return 1;
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = true;
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, visualization);

    auto node = std::make_shared<StereoSlamNode>(&SLAM, argv[2], argv[3]);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
