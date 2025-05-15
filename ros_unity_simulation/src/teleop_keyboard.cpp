#include "rclcpp/rclcpp.hpp"
#include "zero_interfaces/msg/control_input.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>

#define LINEAR_STEP 0.1
#define ANGULAR_STEP 0.1

class TeleopKeyboard : public rclcpp::Node
{
public:
    TeleopKeyboard()
    : Node("teleop_keyboard"), linear_vel_(0.0), angular_vel_(0.0)
    {
        publisher_ = this->create_publisher<zero_interfaces::msg::ControlInput>("control_input", 10);
        RCLCPP_INFO(this->get_logger(), "Teleop Keyboard Initialized. Use W/A/S/D to control, Q to stop.");

        // Keyboard input loop
        while (rclcpp::ok()) {
            char key = getKeyPress();
            handleKeyInput(key);
            publishControlInput();
        }
    }

private:
    double linear_vel_;
    double angular_vel_;
    rclcpp::Publisher<zero_interfaces::msg::ControlInput>::SharedPtr publisher_;

    char getKeyPress()
    {
        char buf = 0;
        struct termios old;
        tcgetattr(0, &old);
        if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
            perror("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror("tcsetattr ~ICANON");
        return buf;
    }

    void handleKeyInput(char key)
    {
        switch (key)
        {
        case 'w':
            linear_vel_ += LINEAR_STEP;
            break;
        case 's':
            linear_vel_ -= LINEAR_STEP;
            break;
        case 'a':
            angular_vel_ += ANGULAR_STEP;
            break;
        case 'd':
            angular_vel_ -= ANGULAR_STEP;
            break;
        case 'q':
            linear_vel_ = 0.0;
            angular_vel_ = 0.0;
            break;
        default:
            break;
        }

        RCLCPP_INFO(this->get_logger(), "Linear Vel: %.2f, Angular Vel: %.2f", linear_vel_, angular_vel_);
    }

    void publishControlInput()
    {
        auto msg = zero_interfaces::msg::ControlInput();
        msg.v = linear_vel_;
        msg.w = angular_vel_;
        publisher_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyboard>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
