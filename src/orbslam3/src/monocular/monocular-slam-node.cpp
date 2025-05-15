#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("mono")
{
    std::cout << " Made mono node - making subscriber " << std::endl;
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    std::cout << " Destructing mono node " << std::endl;
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    RCLCPP_DEBUG(this->get_logger(), "Grabbing image");
    std::cout << " Grabbing image " << std::endl;
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent: t="<<msg->header.stamp.sec<<std::endl;
    
    // Sanity checks
    if (!m_SLAM) {
        RCLCPP_ERROR(this->get_logger(), "SLAM system is null!");
        return;
    }
    if (m_cvImPtr->image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Empty image received!");
        return;
    }

    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
}
