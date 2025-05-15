#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class MonocularInertialNode : public rclcpp::Node
{
public:
    MonocularInertialNode(ORB_SLAM3::System* pSLAM);
    ~MonocularInertialNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();

    // Subscribers
    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;

    // System
    ORB_SLAM3::System* m_SLAM;

    // Separate thread for synchronising IMU/Image messages and passing into tracker
    std::thread *syncThread_;  

    // IMU buffer
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutexIMU_;

    // Camera buffer
    queue<ImageMsg::SharedPtr> imgBuf_;
    std::mutex bufMutexImg_;
};

#endif
