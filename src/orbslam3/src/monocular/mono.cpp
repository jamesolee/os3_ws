#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

#include "System.h"


int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }
    std::cout << " Initializing rclcpp " << std::endl;
    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool visualization = true;
    std::cout << "Making System" << std::endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);

    std::cout << "Making mono node " << std::endl;
    auto node = std::make_shared<MonocularSlamNode>(&SLAM);
    RCLCPP_DEBUG(node->get_logger(), "Made mono node shared");
    std::cout << "============================ " << std::endl;

    RCLCPP_DEBUG(node->get_logger(), "Shutting down mono node");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
