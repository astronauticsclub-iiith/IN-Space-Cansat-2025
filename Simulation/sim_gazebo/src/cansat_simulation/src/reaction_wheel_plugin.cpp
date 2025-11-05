#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace gazebo
{
  class ReactionWheelPlugin : public ModelPlugin
  {
    private: physics::ModelPtr model;
    private: physics::JointPtr joint_x, joint_y, joint_z;
    private: rclcpp::Node::SharedPtr ros_node;
    private: rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_x, sub_y, sub_z;
    private: std::thread ros_thread;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;
      this->joint_x = this->model->GetJoint("rw_x_joint");
      this->joint_y = this->model->GetJoint("rw_y_joint");
      this->joint_z = this->model->GetJoint("rw_z_joint");

      // Initialize ROS node if it hasn't been already
      if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
      }
      this->ros_node = rclcpp::Node::make_shared("reaction_wheel_controller");

      // Create subscribers with lambda callbacks
      sub_x = this->ros_node->create_subscription<std_msgs::msg::Float64>(
          "/cansat/rw_x_Force", 10,
          [this](const std_msgs::msg::Float64::SharedPtr msg) {
            this->joint_x->SetForce(0, msg->data);
          });

      sub_y = this->ros_node->create_subscription<std_msgs::msg::Float64>(
          "/cansat/rw_y_Force", 10,
          [this](const std_msgs::msg::Float64::SharedPtr msg) {
            this->joint_y->SetForce(0, msg->data);
          });

      sub_z = this->ros_node->create_subscription<std_msgs::msg::Float64>(
          "/cansat/rw_z_Force", 10,
          [this](const std_msgs::msg::Float64::SharedPtr msg) {
            this->joint_z->SetForce(0, msg->data);
          });

      this->ros_thread = std::thread([this]() { rclcpp::spin(this->ros_node); });
      gzmsg << "Reaction Wheel Plugin Loaded." << std::endl;
    }

    public: ~ReactionWheelPlugin()
    {
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
      if (this->ros_thread.joinable()) {
        this->ros_thread.join();
      }
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(ReactionWheelPlugin)
}