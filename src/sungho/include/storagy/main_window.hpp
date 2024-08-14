#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QMessageBox>
#include <QImage>
#include <QLabel>
#include <thread>  // 스레드를 사용하기 위해 추가
#include "storagy/image_sub_node.hpp"
#include "storagy/battery_sub_node.hpp"
#include "storagy/robot_control_node.hpp"

#include "teleop_window.hpp" 

class TeleopWindow;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(std::shared_ptr<ImageSubscriberNode> image_node,
                        std::shared_ptr<BatterySubscriberNode> battery_node,
                        std::shared_ptr<RobotControlNode> robot_node,
                        QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void rgbImageReceived(const QImage& image);
    void depthImageReceived(const QImage& image);
    void batteryVoltageReceived(const QString& battery);
    void robotPoseReceived(const RobotControlNode::RobotPose& pose);

private slots:
    void updateRgbImage(const QImage& image);
    void updateDepthImage(const QImage& image);
    void updateBatteryVoltage(const QString& battery);
    void updateRobotPose(const RobotControlNode::RobotPose& pose);
    void goalBtnClicked();
    void cancelbtnClicked();
    void setPoseBtnClicked();
    void onRgbButtonClicked();
    void onDepthButtonClicked();
    void onTeleopButtonClicked();

    
private:
    Ui::MainWindow *ui;
    std::shared_ptr<ImageSubscriberNode> image_node_;
    std::shared_ptr<BatterySubscriberNode> battery_node_;
    std::shared_ptr<RobotControlNode> robot_node_;

    // 스레드 변수 선언
    std::thread ros_image_thread_;
    std::thread ros_battery_thread_;
    std::thread ros_robot_thread_;

    TeleopWindow* teleop_window_;

    void updateImageLabel(QLabel* label, const QImage& image);
    void setupConnections();
};

Q_DECLARE_METATYPE(RobotControlNode::RobotPose)

#endif // MAIN_WINDOW_HPP
