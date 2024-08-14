#ifndef IMAGE_SUBSCRIBER_NODE_HPP
#define IMAGE_SUBSCRIBER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>

class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode();

    std::function<void(const cv::Mat&)> rgb_image_callback;
    std::function<void(const cv::Mat&)> depth_image_callback;

    cv::Mat getRgbImage();
    cv::Mat getDepthImage();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    
    void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    cv::Mat rgb_image_;
    cv::Mat depth_image_;
    std::mutex rgb_mutex_;
    std::mutex depth_mutex_;
};

#endif // IMAGE_SUBSCRIBER_NODE_HPP