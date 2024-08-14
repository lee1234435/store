#ifndef TELEOP_WINDOW_HPP
#define TELEOP_WINDOW_HPP

#include <QWidget>
#include <QKeyEvent>
#include <memory>
#include "storagy/robot_control_node.hpp"

namespace Ui {
class TeleopWindow;
}

class TeleopWindow : public QWidget
{
    Q_OBJECT

public:
    explicit TeleopWindow(std::shared_ptr<RobotControlNode> robot_node, QWidget *parent = nullptr);
    ~TeleopWindow();

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;

private slots:
    void onForwardPressed();
    void onForwardReleased();
    void onBackwardPressed();
    void onBackwardReleased();
    void onLeftPressed();
    void onLeftReleased();
    void onRightPressed();
    void onRightReleased();
    void onStopClicked();
    void onLinearVelocityChanged(int value);
    void onAngularVelocityChanged(int value);

private:
    Ui::TeleopWindow *ui; 
    std::shared_ptr<RobotControlNode> robot_node_;
    
    double linear_velocity_;
    double angular_velocity_;
    double max_linear_velocity_;
    double max_angular_velocity_;

    static constexpr double LINEAR_STEP = 0.1;
    static constexpr double ANGULAR_STEP = 0.1;
    static constexpr double MAX_LINEAR_VELOCITY = 1.0;  
    static constexpr double MAX_ANGULAR_VELOCITY = 1.0;  

    void sendVelocityCommand();
    void updateVelocityLabels();
    void updateStatusLabel();
    void setLinearVelocity(double velocity);
    void setAngularVelocity(double velocity);
};

#endif // TELEOP_WINDOW_HPP