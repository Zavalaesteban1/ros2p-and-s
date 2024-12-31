// string library
#include <string>
// time library
#include <chrono>
// library used to manage dynamic memory
#include <memory>
#include <iostream>
//we will send std msgs of the type string, that is, we will send strings
#include "std_msgs/msg/string.hpp"
//C++ ROS Client Library API
#include "rclcpp/rclcpp.hpp"

// we use this namespace
using namespace std::chrono_literals;

// here we embed the publisher in a C++ class that is inherited from ROS2 Node class
class SimpleSubscriber : public rclcpp::Node
{
    public:
    // constructor
    SimpleSubscriber() : Node("subscriber_node")
    {
        // lambda function whose purpose is similar to the lambda function in the subscriber node 
        auto callBackFunction =
        [this](std_msgs::msg::String::UniquePtr msg) -> void 
        {
            // here, we just print the message in the terminal window
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        };

        // create subscription
        subscriberObject = this->create_subscription<std_msgs::msg::String>(
            "communication_topic", 
            10, 
            callBackFunction
        );
    }
    
    private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriberObject;
};

int main(int argc, char * argv[])
{
    // initialize ROS2
    rclcpp::init(argc, argv);
    // spins the node - the callback function is called
    rclcpp::spin(std::make_shared<SimpleSubscriber>());
    // shutdown after the user presses CTRL+C
    rclcpp::shutdown();
    return 0;
}
