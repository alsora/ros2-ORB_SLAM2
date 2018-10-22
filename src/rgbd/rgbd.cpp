#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include"System.h"

using namespace std;

using namespace std::placeholders;

using ImageMsg = sensor_msgs::msg::Image;

rclcpp::Node::SharedPtr g_node = nullptr;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("orbslam");

    if(argc != 3)
    {
        cerr << endl << "Usage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings" << endl;        
        rclcpp::shutdown();
        return 1;
    }

    bool visualization = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, visualization);
    ImageGrabber igb(&SLAM);

    // create subscribers to the topics of interest
    message_filters::Subscriber<ImageMsg> rgb_sub(g_node, "camera/rgb");
    message_filters::Subscriber<ImageMsg> depth_sub(g_node, "camera/depth");

    // define a message synchronization policy
    typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg> approximate_sync_policy;
    message_filters::Synchronizer<approximate_sync_policy>syncApproximate(approximate_sync_policy(10), rgb_sub, depth_sub);

    // register the synchronization callback
    syncApproximate.registerCallback(&ImageGrabber::GrabRGBD, &igb);

    rclcpp::spin(g_node);

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();

    g_node = nullptr;

    return 0;
}

void ImageGrabber::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    
    // Copy the ros rgb image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(g_node->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(g_node->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    

    mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, msgRGB->header.stamp.sec);
    
}

