#include "rclcpp/rclcpp.hpp"
#include "zero_interfaces/msg/cmd_vel.hpp"
#include "zero_interfaces/msg/control_input.hpp"
#include <functional>
#include <chrono>
#include <cmath>

#define WHEEL_R  0.09394
#define WHEELBASE  0.6638
#define THREAD 0.517

using std::placeholders::_1;
using namespace std::chrono_literals;

class ZeroController : public rclcpp::Node
{
public:
    ZeroController()
    : Node("zero_controller")
    {
        publisher_ = this->create_publisher<zero_interfaces::msg::CmdVel>("vel_ang", 10);
        subscriber_ = this->create_subscription<zero_interfaces::msg::ControlInput>(
            "control_input", 10, std::bind(&ZeroController::kinematics_callback, this, _1));
    }

private:
    float v = 0.0f;
    float w = 0.0f;
    float theta_l = 0.0f;
    float theta_r = 0.0f;

    void kinematics_callback(const zero_interfaces::msg::ControlInput::SharedPtr msg)
    {
        v = msg->v;
        w = msg->w;

        RCLCPP_INFO(this->get_logger(), "Received ControlInput - v: %f, w: %f", v, w);
        
        float v_left = sqrt((v - (THREAD / 2.0f) * w) * (v - (THREAD / 2.0f) * w) + ((w * WHEELBASE) / 2) * ((w * WHEELBASE) / 2));
        float v_right = sqrt((v + (THREAD / 2.0f) * w) * (v - (THREAD / 2.0f) * w) + ((w * WHEELBASE) / 2) * ((w * WHEELBASE) / 2));
        
        float u_left = v_left / WHEEL_R;
        float u_right = v_right / WHEEL_R;

        theta_l = atan2(WHEELBASE, 2 * ((v / w) - (THREAD / 2.0f)));
        theta_r = atan2(WHEELBASE, 2 * ((v / w) + (THREAD / 2.0f)));

        auto cmd_vel_msg = zero_interfaces::msg::CmdVel();

        cmd_vel_msg.fl_vel = u_left;
        cmd_vel_msg.bl_vel = u_left;
        cmd_vel_msg.fr_vel = u_right;
        cmd_vel_msg.br_vel = u_right;

        if (w > 0.0f) {  // Robot spins CCW (+)
            cmd_vel_msg.fl_ang = -theta_l;   // Front Left   CCW
            cmd_vel_msg.bl_ang = theta_l;   // Back Left    CW
            cmd_vel_msg.fr_ang = -theta_r;   // Front Right  CCW
            cmd_vel_msg.br_ang = theta_r;   // Back Right   CW
          }
          else if (w < 0.0f) {  // Robot spins CW (-)
            cmd_vel_msg.fl_ang = theta_l;   // Front Left   CW
            cmd_vel_msg.bl_ang = -theta_l;   // Back Left    CCW
            cmd_vel_msg.fr_ang = theta_r;   // Front Right  CW
            cmd_vel_msg.br_ang = -theta_r;   // Back Right   CCW
        }

        RCLCPP_INFO(this->get_logger(), 
                    "Publishing CmdVel - Velocities [FL: %f, BL: %f, FR: %f, BR: %f], Angles [FL: %f, BL: %f, FR: %f, BR: %f]", 
                    cmd_vel_msg.fl_vel, cmd_vel_msg.bl_vel, 
                    cmd_vel_msg.fr_vel, cmd_vel_msg.br_vel, 
                    cmd_vel_msg.fl_ang, cmd_vel_msg.bl_ang, 
                    cmd_vel_msg.fr_ang, cmd_vel_msg.br_ang);
        publisher_->publish(cmd_vel_msg);
    }

    rclcpp::Publisher<zero_interfaces::msg::CmdVel>::SharedPtr publisher_;
    rclcpp::Subscription<zero_interfaces::msg::ControlInput>::SharedPtr subscriber_;
};
   
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZeroController>());
  rclcpp::shutdown();
  return 0;
}