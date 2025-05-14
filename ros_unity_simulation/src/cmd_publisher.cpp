#include "rclcpp/rclcpp.hpp"
#include "zero_interfaces/msg/cmd_vel.hpp"
#include <chrono>

using namespace std::chrono_literals;

class CmdPublisher : public rclcpp::Node
{
public:
    CmdPublisher()
    : Node("cmd_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<zero_interfaces::msg::CmdVel>("vel_ang", 10);
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&CmdPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = zero_interfaces::msg::CmdVel();

        message.fl_vel = 1.0;
        message.bl_vel = 1.1;
        message.fr_vel = 1.2;
        message.br_vel = 1.3;

        message.fl_ang = 0.1;
        message.bl_ang = 0.2;
        message.fr_ang = 0.3;
        message.br_ang = 0.4;

        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing velocities & angles: "
        << "FL_Vel: " << message.fl_vel << ", "
        << "BL_Vel: " << message.bl_vel << ", "
        << "FR_Vel: " << message.fr_vel << ", "
        << "BR_Vel: " << message.br_vel << ", "
        << "FL_Ang: " << message.fl_ang << ", "
        << "BL_Ang: " << message.bl_ang << ", "
        << "FR_Ang: " << message.fr_ang << ", "
        << "BR_Ang: " << message.br_ang);

        publisher_->publish(message);
    }

    rclcpp::Publisher<zero_interfaces::msg::CmdVel>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};
   
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdPublisher>());
  rclcpp::shutdown();
  return 0;
}