#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>

#include"System.h"

using namespace std;

using std::placeholders::_1;

using ImageMsg = sensor_msgs::msg::Image;

rclcpp::Node::SharedPtr g_node = nullptr;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const ImageMsg::SharedPtr msg);

    cv::Mat imageMsgToCVMat(ImageMsg::SharedPtr ros_image) const;

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("orbslam");

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        rclcpp::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = rmw_qos_profile_default.depth;
    custom_qos_profile.reliability = rmw_qos_profile_default.reliability;
    custom_qos_profile.history = rmw_qos_profile_default.history;
    rclcpp::Subscription<ImageMsg>::SharedPtr subscriber = g_node->create_subscription<ImageMsg>("camera", std::bind(&ImageGrabber::GrabImage, igb, _1));
    
 
    rclcpp::spin(g_node);

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();

    g_node = nullptr;

    return 0;
}

void ImageGrabber::GrabImage(const ImageMsg::SharedPtr msg)
{
    
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(g_node->get_logger(), "cv_bridge exception: %s", e.what());
        return;
}
    
    mpSLAM->TrackMonocular(cv_ptr->image, msg->header.stamp.sec);
    
}


