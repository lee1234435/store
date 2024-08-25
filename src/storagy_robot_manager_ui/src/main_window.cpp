#include <thread>
#include <QDebug>
#include "storagy/main_window.hpp"
#include "ui_main_window.h"

MainWindow::MainWindow(std::shared_ptr<ImageSubscriberNode> image_node,
                       std::shared_ptr<BatterySubscriberNode> battery_node,
                       std::shared_ptr<RobotControlNode> robot_node,
                       QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), 
      image_node_(image_node), battery_node_(battery_node), robot_node_(robot_node), teleop_window_(nullptr)
{
    qRegisterMetaType<RobotControlNode::RobotPose>("RobotControlNode::RobotPose");

    ui->setupUi(this);
    setupConnections();

    qDebug() << "MainWindow constructor: Starting ROS threads";

    // ROS 콜백을 처리할 쓰레드 시작
    ros_image_thread_ = std::thread([this]() {
        qDebug() << "Image thread started";
        rclcpp::spin(image_node_);
    });

    ros_battery_thread_ = std::thread([this]() {
        qDebug() << "Battery thread started";
        rclcpp::spin(battery_node_);
    });

    ros_robot_thread_ = std::thread([this]() {
        qDebug() << "Robot thread started";
        rclcpp::spin(robot_node_);
    });
}

MainWindow::~MainWindow()
{
    // ROS 노드 스핀을 중지하고 쓰레드를 종료
    rclcpp::shutdown();
    if (ros_image_thread_.joinable()) {
        ros_image_thread_.join();
    }
    if (ros_battery_thread_.joinable()) {
        ros_battery_thread_.join();
    }
    if (ros_robot_thread_.joinable()) {
        ros_robot_thread_.join();
    }

    if (teleop_window_) {
        delete teleop_window_;
    }

    delete ui;
}

void MainWindow::setupConnections()
{
    qDebug() << "Setting up connections";

    image_node_->rgb_image_callback = [this](const cv::Mat& cv_image) {
        qDebug() << "RGB Image callback called";
        QImage qimg(cv_image.data, cv_image.cols, cv_image.rows, cv_image.step, QImage::Format_RGB888);
        emit rgbImageReceived(qimg.rgbSwapped());
    };

    image_node_->depth_image_callback = [this](const cv::Mat& cv_image) {
        qDebug() << "Depth Image callback called";
        QImage qimg(cv_image.data, cv_image.cols, cv_image.rows, cv_image.step, QImage::Format_RGB888);
        emit depthImageReceived(qimg);
    };

    battery_node_->battery_callback = [this](const std::string& voltage) {
        qDebug() << "Battery callback called";
        emit batteryVoltageReceived(QString::fromStdString(voltage));
    };

    robot_node_->pose_callback = [this](const RobotControlNode::RobotPose& pose) {
        qDebug() << "Robot pose callback called";
        emit robotPoseReceived(pose);
    };

    // 시그널-슬롯 연결 추가
    connect(this, &MainWindow::rgbImageReceived, this, &MainWindow::updateRgbImage, Qt::QueuedConnection);
    connect(this, &MainWindow::depthImageReceived, this, &MainWindow::updateDepthImage, Qt::QueuedConnection);
    connect(this, &MainWindow::batteryVoltageReceived, this, &MainWindow::updateBatteryVoltage, Qt::QueuedConnection);
    connect(this, &MainWindow::robotPoseReceived, this, &MainWindow::updateRobotPose, Qt::QueuedConnection);

    connect(ui->goal_btn, &QPushButton::clicked, this, &MainWindow::goalBtnClicked);
    connect(ui->cancel_btn, &QPushButton::clicked, this, &MainWindow::cancelbtnClicked);

    connect(ui->set_pos_btn, &QPushButton::clicked, this, &MainWindow::setPoseBtnClicked);

    connect(ui->rgb_rbtn, &QRadioButton::clicked, this, &MainWindow::onRgbButtonClicked);
    connect(ui->depth_rbtn, &QRadioButton::clicked, this, &MainWindow::onDepthButtonClicked);
    connect(ui->teleop_btn, &QPushButton::clicked, this, &MainWindow::onTeleopButtonClicked);


    qDebug() << "Connections setup completed";
}


void MainWindow::updateRgbImage(const QImage& image)
{
    qDebug() << "updateRgbImage called. Original image size:" << image.size();
    updateImageLabel(ui->image_label, image);
}

void MainWindow::updateDepthImage(const QImage& image)
{
    qDebug() << "updateDepthImage called. Original image size:" << image.size();
    updateImageLabel(ui->depth_label, image);
}

void MainWindow::updateBatteryVoltage(const QString& battery)
{
    qDebug() << "updateBatteryVoltage called";
    ui->battery_line->setText(battery);
}

void MainWindow::updateImageLabel(QLabel* label, const QImage& image)
{
    QSize labelSize = label->size();
    QPixmap pixmap = QPixmap::fromImage(image);
    QPixmap scaledPixmap = pixmap.scaled(labelSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    label->setPixmap(scaledPixmap);
    label->setAlignment(Qt::AlignCenter);
    qDebug() << "Scaled image size:" << scaledPixmap.size();
}


void MainWindow::updateRobotPose(const RobotControlNode::RobotPose& pose)
{
    qDebug() << "updateRobotPose called with values:"
             << "x:" << pose.x
             << "y:" << pose.y
             << "degree:" << pose.degree
             << "theta:" << pose.theta;
    
    if (ui->coor_x_line && ui->coor_y_line && ui->coor_degree_line && ui->coor_theta_line) {
        ui->coor_x_line->setText(QString::number(pose.x, 'f', 6));
        ui->coor_y_line->setText(QString::number(pose.y, 'f', 6));
        ui->coor_degree_line->setText(QString::number(pose.degree, 'f', 6));
        ui->coor_theta_line->setText(QString::number(pose.theta, 'f', 6));
    } else {
        qDebug() << "One or more UI elements are null";
    }
}

void MainWindow::goalBtnClicked()
{
    qDebug() << "Set Goal button clicked";

    bool ok;
    double x = ui->goal_x_line->text().toDouble(&ok);
    if (!ok) {
        qDebug() << "Invalid x coordinate";
        return;
    }

    double y = ui->goal_y_line->text().toDouble(&ok);
    if (!ok) {
        qDebug() << "Invalid y coordinate";
        return;
    }

    double degree = ui->goal_degree_line->text().toDouble(&ok);
    if (!ok) {
        qDebug() << "Invalid degree";
        return;
    }

    // 각도를 라디안으로 변환
    double theta = degree * M_PI / 180.0;

    qDebug() << "Navigating to goal: x =" << x << ", y =" << y << ", theta =" << theta;

    // RobotControlNode의 navigateToPose 함수 호출
    robot_node_->navigateToPose(x, y, theta);
}

void MainWindow::cancelbtnClicked()
{
    qDebug() << "Cancel Goal button clicked";

    // RobotControlNode의 cancelNavigation 함수 호출
    robot_node_->cancelNavigation();
}

void MainWindow::setPoseBtnClicked()
{
    bool ok;
    double x = ui->goal_x_line->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Invalid Input", "Invalid X coordinate");
        return;
    }

    double y = ui->goal_y_line->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Invalid Input", "Invalid Y coordinate");
        return;
    }

    double degree = ui->goal_degree_line->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Invalid Input", "Invalid degree");
        return;
    }

    robot_node_->setInitialPose(x, y, degree);

    QMessageBox::information(this, "Position Set", "Initial position has been set");
}

void MainWindow::onRgbButtonClicked()
{
    ui->stackedWidget->setCurrentIndex(0);
}

void MainWindow::onDepthButtonClicked()
{
    ui->stackedWidget->setCurrentIndex(1);
}

void MainWindow::onTeleopButtonClicked()
{
    if (!teleop_window_) {
        teleop_window_ = new TeleopWindow(robot_node_, nullptr); // 부모를 nullptr로 설정
    }
    teleop_window_->show();
    teleop_window_->raise();
    teleop_window_->activateWindow();
}
