#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

#include"ORB_SLAM2/MapPoint.h"

using ImageMsg = sensor_msgs::msg::Image;
using MarkerMsg = visualization_msgs::msg::Marker;

using namespace std;

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM2::System* pSLAM, const string &strVocFile, const string &strSettingsFile)
:   Node("orbslam")
    ,m_SLAM(pSLAM)
{

    m_image_subscriber = this->create_subscription<ImageMsg>("camera", std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));


    m_annotated_image_publisher = this->create_publisher<ImageMsg>("annotated_frame");

    mState = ORB_SLAM2::Tracking::SYSTEM_NOT_READY;

    //mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    
    mbUpdated = true;
    std::cout<<"publisher created"<<std::endl;

}


MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

}


void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    std::cout<<"Grab image"<<std::endl;   
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Mat Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, msg->header.stamp.sec);
    std::cout<<"TrackMonocular DONE"<<std::endl;   

    // get SLAM data
    //int t_state = m_SLAM->GetTrackingState();
    //vector<MapPoint*> map_points = m_SLAM->GetTrackedMapPoints();
    //vector<cv::KeyPoint> key_points =  m_SLAM->GetTrackedKeyPointsUn();
   

    UpdateSLAMState();

}


void MonocularSlamNode::UpdateSLAMState()
{
 
    unique_lock<mutex> lock(mMutex);
    /*
    ORB_SLAM2::Frame currentFrame = m_SLAM->GetCurrentFrame();
    mState = m_SLAM->GetTrackingState();
  
    mvCurrentKeys = currentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    //mbOnlyTracking = pTracker->mbOnlyTracking;

    if (mState == ORB_SLAM2::Tracking::NOT_INITIALIZED){
        
        mvIniKeys = m_SLAM->GetInitialKeys();
        mvIniMatches = m_SLAM->GetInitialMatches();
    }
    else if(mState == ORB_SLAM2::Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            ORB_SLAM2::MapPoint* pMP = currentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!currentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }

    mbUpdated = true;
    */
}