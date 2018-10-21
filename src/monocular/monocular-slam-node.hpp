
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <cv_bridge/cv_bridge.h>

#include"System.h"
#include"Frame.h"
#include "Map.h"
#include "Tracking.h"


class MonocularSlamNode : public rclcpp::Node
{

public:

    MonocularSlamNode(ORB_SLAM2::System* pSLAM, const string &strVocFile, const string &strSettingsFile);


    ~MonocularSlamNode();


private: 


    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    void UpdateSLAMState();

    ORB_SLAM2::System* m_SLAM;

    std::mutex mMutex;

    cv_bridge::CvImagePtr m_cvImPtr;
    
    
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    
    
    int mState;
    bool mbUpdated;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_annotated_image_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_map_publisher;

};