#include "storagy/image_sub_node.hpp"

ImageSubscriberNode::ImageSubscriberNode()
: Node("image_subscriber_node")
{
    rgb_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 10, 
        std::bind(&ImageSubscriberNode::rgbImageCallback, this, std::placeholders::_1));

    depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_raw", 10, 
        std::bind(&ImageSubscriberNode::depthImageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Image subscriber node has been initialized.");
}

void ImageSubscriberNode::rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        {
            std::lock_guard<std::mutex> lock(rgb_mutex_);
            rgb_image_ = cv_ptr->image;
        }
        
        if (rgb_image_callback) {
            rgb_image_callback(rgb_image_);
        }

        RCLCPP_INFO(this->get_logger(), "Received an RGB image. Size: %dx%d", rgb_image_.cols, rgb_image_.rows);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void ImageSubscriberNode::depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat normalized;
        cv::normalize(cv_ptr->image, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::Mat colorized;
        cv::applyColorMap(normalized, colorized, cv::COLORMAP_JET);
        
        {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            depth_image_ = colorized;
        }
        
        if (depth_image_callback) {
            depth_image_callback(depth_image_);
        }

        RCLCPP_INFO(this->get_logger(), "Received a Depth image. Size: %dx%d", depth_image_.cols, depth_image_.rows);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not convert depth image.");
    }
}

cv::Mat ImageSubscriberNode::getRgbImage()
{
    std::lock_guard<std::mutex> lock(rgb_mutex_);
    return rgb_image_.clone();
}

cv::Mat ImageSubscriberNode::getDepthImage()
{
    std::lock_guard<std::mutex> lock(depth_mutex_);
    return depth_image_.clone();
}