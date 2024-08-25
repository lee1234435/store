#include "storagy/teleop_window.hpp"
#include "ui_teleop_window.h"
#include <QDebug>
#include <algorithm>

TeleopWindow::TeleopWindow(std::shared_ptr<RobotControlNode> robot_node, QWidget *parent)
    : QWidget(parent), ui(new Ui::TeleopWindow), robot_node_(robot_node),
      linear_velocity_(0.0), angular_velocity_(0.0),
      max_linear_velocity_(MAX_LINEAR_VELOCITY), max_angular_velocity_(MAX_ANGULAR_VELOCITY)
{
    ui->setupUi(this);
    setFocusPolicy(Qt::StrongFocus);

    // 버튼 연결
    connect(ui->forwardButton, &QPushButton::pressed, this, &TeleopWindow::onForwardPressed);
    connect(ui->forwardButton, &QPushButton::released, this, &TeleopWindow::onForwardReleased);
    connect(ui->backwardButton, &QPushButton::pressed, this, &TeleopWindow::onBackwardPressed);
    connect(ui->backwardButton, &QPushButton::released, this, &TeleopWindow::onBackwardReleased);
    connect(ui->leftButton, &QPushButton::pressed, this, &TeleopWindow::onLeftPressed);
    connect(ui->leftButton, &QPushButton::released, this, &TeleopWindow::onLeftReleased);
    connect(ui->rightButton, &QPushButton::pressed, this, &TeleopWindow::onRightPressed);
    connect(ui->rightButton, &QPushButton::released, this, &TeleopWindow::onRightReleased);
    connect(ui->stopButton, &QPushButton::clicked, this, &TeleopWindow::onStopClicked);

    // 슬라이더 설정 및 연결
    ui->linearVelocitySlider->setRange(0, 100);  // 0% to 100%
    ui->angularVelocitySlider->setRange(0, 100);  // 0% to 100%
    ui->linearVelocitySlider->setValue(50);
    ui->angularVelocitySlider->setValue(50);

    connect(ui->linearVelocitySlider, &QSlider::valueChanged, this, &TeleopWindow::onLinearVelocityChanged);
    connect(ui->angularVelocitySlider, &QSlider::valueChanged, this, &TeleopWindow::onAngularVelocityChanged);

    updateVelocityLabels();
    updateStatusLabel();
}

TeleopWindow::~TeleopWindow()
{
    delete ui;
}

void TeleopWindow::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
        case Qt::Key_W: onForwardPressed(); break;
        case Qt::Key_S: onBackwardPressed(); break;
        case Qt::Key_A: onLeftPressed(); break;
        case Qt::Key_D: onRightPressed(); break;
        case Qt::Key_Space: onStopClicked(); break;
    }
}

void TeleopWindow::keyReleaseEvent(QKeyEvent *event)
{
    switch (event->key()) {
        case Qt::Key_W: onForwardReleased(); break;
        case Qt::Key_S: onBackwardReleased(); break;
        case Qt::Key_A: onLeftReleased(); break;
        case Qt::Key_D: onRightReleased(); break;
    }
}

void TeleopWindow::onForwardPressed() { setLinearVelocity(max_linear_velocity_); }
void TeleopWindow::onForwardReleased() { setLinearVelocity(0.0); }
void TeleopWindow::onBackwardPressed() { setLinearVelocity(-max_linear_velocity_); }
void TeleopWindow::onBackwardReleased() { setLinearVelocity(0.0); }
void TeleopWindow::onLeftPressed() { setAngularVelocity(max_angular_velocity_); }
void TeleopWindow::onLeftReleased() { setAngularVelocity(0.0); }
void TeleopWindow::onRightPressed() { setAngularVelocity(-max_angular_velocity_); }
void TeleopWindow::onRightReleased() { setAngularVelocity(0.0); }
void TeleopWindow::onStopClicked() { setLinearVelocity(0.0); setAngularVelocity(0.0); }

void TeleopWindow::onLinearVelocityChanged(int value)
{
    max_linear_velocity_ = (value / 100.0) * MAX_LINEAR_VELOCITY;
    updateVelocityLabels();
    updateStatusLabel();
}

void TeleopWindow::onAngularVelocityChanged(int value)
{
    max_angular_velocity_ = (value / 100.0) * MAX_ANGULAR_VELOCITY;
    updateVelocityLabels();
    updateStatusLabel();
}

void TeleopWindow::updateVelocityLabels()
{
    ui->linearVelocityValue->setText(QString::number(max_linear_velocity_, 'f', 2));
    ui->angularVelocityValue->setText(QString::number(max_angular_velocity_, 'f', 2));
}

void TeleopWindow::setLinearVelocity(double velocity)
{
    linear_velocity_ = std::clamp(velocity, -max_linear_velocity_, max_linear_velocity_);
    sendVelocityCommand();
    updateStatusLabel();
}

void TeleopWindow::setAngularVelocity(double velocity)
{
    angular_velocity_ = std::clamp(velocity, -max_angular_velocity_, max_angular_velocity_);
    sendVelocityCommand();
    updateStatusLabel();
}

void TeleopWindow::sendVelocityCommand()
{
    try {
        qDebug() << "Sending velocity command: linear =" << linear_velocity_ << ", angular =" << angular_velocity_;
        robot_node_->sendVelocityCommand(linear_velocity_, angular_velocity_);
    } catch (const std::exception& e) {
        qDebug() << "Error sending velocity command:" << e.what();
    }
}

void TeleopWindow::updateStatusLabel()
{
    QString status = QString("Linear: %1 / %2 m/s, Angular: %3 / %4 rad/s")
                         .arg(linear_velocity_, 0, 'f', 2)
                         .arg(max_linear_velocity_, 0, 'f', 2)
                         .arg(angular_velocity_, 0, 'f', 2)
                         .arg(max_angular_velocity_, 0, 'f', 2);
    ui->statusLabel->setText(status);
}