#include "monocular-inertial-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System* pSLAM)
:   Node("mono_inertial")
{
    std::cout << " Made mono node - making subscriber " << std::endl;
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>("camera",100,std::bind(&MonocularInertialNode::GrabImage, this, _1));
    m_imu_subscriber = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&MonocularInertialNode::GrabImu, this, _1));
    syncThread_ = new std::thread(&MonocularInertialNode::SyncWithImu, this);
}

MonocularInertialNode::~MonocularInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;
    
    // Stop all threads
    std::cout << " Destructing mono node " << std::endl;
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutexIMU_.lock();
    imuBuf_.push(msg);
    bufMutexIMU_.unlock();
}


vvoid MonocularInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Grabbing image");
    bufMutexImg_.lock();
    imgBuf_.push(msg);
    bufMutexImg_.unlock();
}

cv::Mat MonocularInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonocularInertialNode::SyncWithImu()
{

    while (1)
    {
        cv::Mat img;
        if (!imgBuf_.empty() && !imuBuf_.empty())
        {
            
            vector<ORB_SLAM3::IMU::Point> vImuMeas;

            // Add a single IMU measurement to vIMUMeas vector
            bufMutexIMU_.lock();
            bufMutexImg_.lock();

            double tIMU = Utility::StampToSec(imuBuf_.front()->header.stamp);
            double tImg = Utility::StampToSec(imgBuf_.front()->header.stamp);

            // If the first image time is later than the last imu reading, skip
            if (tImg > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;
            bufMutexIMU_.unlock();
            bufMutexImg_.unlock();

            // Since images are sampled at a faster rate, get the image closest to this IMU measurement
            bufMutexImg_.lock();
            while ((tImg - tIMU) > maxTimeDiff && imgBuf_.size() > 1)
            {
                imgBuf_.pop();
                tImg = Utility::StampToSec(imgBuf_.front()->header.stamp);
            }
            bufMutexImg_.unlock();
            
            // If time diff between img and IMU is still too large, skip
            if ((tImg - tIMU) > maxTimeDiff || (tIMU - tImg) > maxTimeDiff)
            {
                std::cout << "big time difference" << std::endl;
                continue;
            }


            // All timing is relatively synchronised, so go ahead and get image
            bufMutexImg_.lock();
            img = GetImage(imgBuf_.front());
            imgBuf_.pop();
            bufMutexImg_.unlock();

            // Get IMU reading
            bufMutexIMU_.lock()
            cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
            cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
            imuBuf_.pop();
            bufMutexIMU_.unlock();


            m_SLAM->TrackMonocular(img, Utility::StampToSec(msg->header.stamp), vIMUMeas);
            
            // Not sure why this is here, perhaps to let the buffers fill up?
            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

