#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"

#include"System.h"

using namespace std;



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if(argc != 4)
    {
        cerr << endl << "Usage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify" << endl;        
        rclcpp::shutdown();
        return 1;
    }

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = true;
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, visualization);


    auto node = std::make_shared<StereoSlamNode>(&SLAM, argv[1], argv[2], argv[3]);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

