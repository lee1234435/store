#include "storagy/robot_control_node.hpp"

RobotControlNode::RobotControlNode()
: Node("robot_control_node")
{
    amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&RobotControlNode::amclPoseCallback, this, std::placeholders::_1));

    nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this,
        "navigate_to_pose"
    );

    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    // 클라이언트가 사용 가능할 때까지 대기
    while (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the action server.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Navigation action server is available.");

    RCLCPP_INFO(this->get_logger(), "Robot node has been initialized.");

}

void RobotControlNode::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_pose_.theta = yaw;
    current_pose_.degree = yaw * 180.0 / M_PI;

    RCLCPP_INFO(this->get_logger(), "Robot Pose: x=%.2f, y=%.2f, theta=%.2f rad, degree=%.2f°", 
                current_pose_.x, current_pose_.y, current_pose_.theta, current_pose_.degree);
    if (pose_callback) {
        RCLCPP_INFO(this->get_logger(), "Calling pose_callback");
        pose_callback(current_pose_);
    }

}

void RobotControlNode::navigateToPose(double x, double y, double degree)
{
    if (!nav_to_pose_client_) {
        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        return;
    }

    if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;
    
    double normalized_degree = normalizeAngle(degree);
    double theta_rad = normalized_degree * M_PI / 180.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_rad);
    q.normalize();

    goal_msg.pose.pose.orientation.x = q.x();
    goal_msg.pose.pose.orientation.y = q.y();
    goal_msg.pose.pose.orientation.z = q.z();
    goal_msg.pose.pose.orientation.w = q.w();


    RCLCPP_INFO(this->get_logger(), "Setting goal: x=%.2f, y=%.2f, theta=%.2f degrees", x, y, normalized_degree);
    RCLCPP_INFO(this->get_logger(), "Orientation quaternion: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                q.x(), q.y(), q.z(), q.w());

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&RobotControlNode::nav_to_pose_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&RobotControlNode::nav_to_pose_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&RobotControlNode::nav_to_pose_result_callback, this, std::placeholders::_1);

        auto future = nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Navigation timed out after 10 seconds");
        nav_to_pose_client_->async_cancel_all_goals();
    }
}

void RobotControlNode::cancelNavigation()
{
    RCLCPP_INFO(this->get_logger(), "Cancelling all navigation goals");
    nav_to_pose_client_->async_cancel_all_goals();
}

void RobotControlNode::nav_to_pose_response_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void RobotControlNode::nav_to_pose_feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f", feedback->distance_remaining);
}

void RobotControlNode::nav_to_pose_result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }
}

double RobotControlNode::normalizeAngle(double angle) {
    angle = fmod(angle, 360.0);
    if (angle > 180.0) {
        angle -= 360.0;
    } else if (angle < -180.0) {
        angle += 360.0;
    }
    return angle;
}


void RobotControlNode::setInitialPose(double x, double y, double degree)
{
    if (!initial_pose_pub_) {
        RCLCPP_ERROR(this->get_logger(), "Initial pose publisher not initialized");
        return;
    }

    try {
        auto message = geometry_msgs::msg::PoseWithCovarianceStamped();

        message.header.frame_id = "map";
        message.header.stamp = this->now();

        message.pose.pose.position.x = x;
        message.pose.pose.position.y = y;
        message.pose.pose.position.z = 0.0;

        double theta_rad = degree * M_PI / 180.0;  // degree to radian 변환
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_rad);
        message.pose.pose.orientation.x = q.x();
        message.pose.pose.orientation.y = q.y();
        message.pose.pose.orientation.z = q.z();
        message.pose.pose.orientation.w = q.w();

        // Covariance 행렬 설정 (예시: 높은 확실성)
        for(size_t i = 0; i < 36; ++i) {
            message.pose.covariance[i] = 0.0;
        }
        message.pose.covariance[0] = 0.25;
        message.pose.covariance[7] = 0.25;
        message.pose.covariance[35] = 0.06853891945200942;

        initial_pose_pub_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published initial pose: x=%.2f, y=%.2f, degree=%.2f", x, y, degree);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in setInitialPose: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown error in setInitialPose");
    }
}

void RobotControlNode::sendVelocityCommand(double linear_velocity, double angular_velocity)
{
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = linear_velocity;
    twist_msg.angular.z = angular_velocity;
    cmd_vel_pub_->publish(twist_msg);
    RCLCPP_INFO(this->get_logger(), "Sending velocity command: linear=%.2f, angular=%.2f", linear_velocity, angular_velocity);
}