#ifndef BATTERY_SUBSCRIBER_NODE_HPP
#define BATTERY_SUBSCRIBER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class BatterySubscriberNode : public rclcpp::Node
{
public:
    BatterySubscriberNode();

    std::function<void(const std::string&)> battery_callback;

private:

    int voltage_int_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr battery_subscription_;
    void batteryCallback(const std_msgs::msg::String::SharedPtr msg);
};

#endif // BATTERY_SUBSCRIBER_NODE_HPP