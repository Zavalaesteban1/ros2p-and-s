// string library
#include <string>
// time library
#include <chrono>
// library used to manage dynamic memory
#include <memory>
#include <iostream>
//we will send std msgs of the type string
#include "std_msgs/msg/string.hpp"
//C++ ROS Client Library API
#include "rclcpp/rclcpp.hpp"

// we use this namespace
using namespace std::chrono_literals;

// here we embed the publisher in a C++ class inherited from ROS2 Node class
class SimplePublisher : public rclcpp::Node
{
    public:
    // constructor
    SimplePublisher() : Node("publisher_node")
    {
        // counterValue is a private variable used to count the number of messages
        counterValue = 0;
        
        // Create publisher object with message type, topic name, and buffer size
        publisherObject = this->create_publisher<std_msgs::msg::String>("communication_topic", 20);
        
        // Callback function that sends the messages
        auto callBackFunction = 
        [this]() -> void 
        {
            // increase the counter
            counterValue++;
            // create an empty message
            auto message = std_msgs::msg::String();
            // fill in the message content
            message.data = "Robot Kill Count " + std::to_string(this->counterValue);
            // print a message to the terminal
            RCLCPP_INFO(this->get_logger(), "Published Message: '%s'", message.data.c_str());
            // publish the message
            publisherObject->publish(message);
        };
        
        // create a timer object that calls the callback function every 1000ms
        timerObject = this->create_wall_timer(1000ms, callBackFunction);
    }
    
    private:
    // timer
    rclcpp::TimerBase::SharedPtr timerObject;
    // publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisherObject;
    // counter
    int counterValue;
};

int main(int argc, char * argv[])
{
    // initialize ROS2
    rclcpp::init(argc, argv);
    // spins the node - the callback function is called
    rclcpp::spin(std::make_shared<SimplePublisher>());
    // shutdown after the user presses CTRL+C
    rclcpp::shutdown();   
    return 0; 
}
