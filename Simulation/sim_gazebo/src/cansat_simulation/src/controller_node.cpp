#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode() : Node("controller_node")
  {
    // Publisher to send torque command to the Z-axis reaction wheel
    publisher_z_ = this->create_publisher<std_msgs::msg::Float64>("/cansat/rw_z_torque", 10);

    // Timer to publish a command periodically
    timer_ = this->create_wall_timer(
      1s, std::bind(&ControllerNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Controller node started. Will publish torque commands.");
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = 0.001; // Apply a small, constant torque to spin the CanSat
    publisher_z_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing torque: %.2f", msg.data);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_z_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}