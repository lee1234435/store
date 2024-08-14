#include "storagy/battery_sub_node.hpp"
#include <iomanip>
#include <sstream>

BatterySubscriberNode::BatterySubscriberNode()
: Node("battery_subscriber_node")
{
    battery_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/battery_voltage", 10, 
        std::bind(&BatterySubscriberNode::batteryCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Battery subscriber node has been initialized.");

}

void BatterySubscriberNode::batteryCallback(const std_msgs::msg::String::SharedPtr msg)
{
    try
    {
        float voltage = std::stof(msg->data);
        voltage_int_ = static_cast<int>(std::round(voltage)); 
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << voltage;
        std::string formatted_voltage = ss.str();

        RCLCPP_INFO(this->get_logger(), "Received battery voltage: %s", formatted_voltage.c_str()); // Log message
        
        if (battery_callback) {
            battery_callback(formatted_voltage);
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error processing battery voltage: %s", e.what());
    }
}