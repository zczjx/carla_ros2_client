#include "rclcpp/rclcpp.hpp"
#include <unistd.h>

int main(int argc, char **argv)
{
    //init client
    rclcpp::init(argc, argv);
    //create new node
    auto node = std::make_shared<rclcpp::Node>("carla_cpp_client");

    while(rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "clarencez: carla client cpp run");
        sleep(1);
    }

    //shutdown
    rclcpp::shutdown();
}