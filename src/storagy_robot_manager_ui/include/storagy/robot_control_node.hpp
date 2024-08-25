#ifndef ROBOT_CONTROL_NODE_HPP
#define ROBOT_CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>

class RobotControlNode : public rclcpp::Node
{
public:
    RobotControlNode();

    struct RobotPose {
        double x;
        double y;
        double theta;
        double degree;
    };  // 세미콜론 추가

    std::function<void(const RobotPose&)> pose_callback;

    void navigateToPose(double x, double y, double degree);
    void cancelNavigation();
    void setInitialPose(double x, double y, double degree);

     void sendVelocityCommand(double linear_velocity, double angular_velocity);

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    RobotPose current_pose_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
     // Navigation2 액션 클라이언트 콜백 함수
    void nav_to_pose_response_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle);
    void nav_to_pose_feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle,
                                       const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    void nav_to_pose_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);

    double normalizeAngle(double angle);

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};


#endif