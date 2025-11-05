#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// A simple node that subscribes to the IMU topic and prints the orientation
class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode() : Node("controller_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&ControllerNode::imu_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Controller node has been started.");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
    // Print the quaternion orientation from the IMU message
    RCLCPP_INFO(this->get_logger(), "IMU Orientation: [w: %.2f, x: %.2f, y: %.2f, z: %.2f]",
      msg->orientation.w,
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z);
  }
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
