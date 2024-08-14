#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "storagy/main_window.hpp"
#include "storagy/image_sub_node.hpp"
#include "storagy/battery_sub_node.hpp"
#include "storagy/robot_control_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // ROS 2 노드 초기화
    QApplication a(argc, argv);  // Qt 애플리케이션 초기화

    // ROS 노드 생성
    auto image_node = std::make_shared<ImageSubscriberNode>();
    auto battery_node = std::make_shared<BatterySubscriberNode>();
    auto robot_control_node = std::make_shared<RobotControlNode>();

    // MainWindow 생성 및 노드 전달
    MainWindow w(image_node, battery_node, robot_control_node);
    w.show();  // 메인 윈도우 표시

    int result = a.exec();  // Qt 이벤트 루프 실행

    // 애플리케이션 종료 시 ROS 노드 종료 및 클린업
    rclcpp::shutdown();

    return result;
}


