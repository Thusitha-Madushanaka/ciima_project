#ifndef CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_
#define CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_

#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "custom_controller_kinematics/custom_controller_kinematics.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"


namespace custom_controller
{

class CustomController : public controller_interface::ControllerInterface
{
public:

  CustomController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn
  on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn
  on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type
  update(
    const rclcpp::Time & current_time,
    const rclcpp::Duration & period) override;

private:

  // ------------------------------------------------------------
  // ROS 2 communication
  // ------------------------------------------------------------

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_subscriber_;

  geometry_msgs::msg::Twist::SharedPtr
    latest_cmd_vel_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
    odom_publisher_;

  std::unique_ptr<tf2_ros::TransformBroadcaster>
    tf_broadcaster_;


  // ------------------------------------------------------------
  // Kinematics
  // ------------------------------------------------------------

  std::unique_ptr<
    custom_controller_kinematics::CustomControllerKinematics>
    kinematics_;


  // ------------------------------------------------------------
  // Odometry
  // ------------------------------------------------------------

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;


  // ------------------------------------------------------------
  // /cmd_vel timeout
  // ------------------------------------------------------------

  double cmd_vel_timeout_ = 0.5;

  rclcpp::Time last_cmd_time_;


  // ------------------------------------------------------------
  // Steering control
  // ------------------------------------------------------------

  // Maximum steering angle change per second.
  //
  // Unit:
  // rad/s
  //
  // Example:
  // 0.5 rad/s means the steering angle can change
  // by at most 0.5 rad every second.

  double max_steering_rate_ = 0.5;


  // Internal steering command currently being sent
  // to the steering joints.

  double commanded_steering_angle_1_ = 0.0;
  double commanded_steering_angle_2_ = 0.0;


  // Maximum allowed difference between the actual
  // steering angle and the desired steering angle
  // before the wheel motors are allowed to move.
  //
  // Unit:
  // rad

  double steering_alignment_tolerance_ = 0.05;
};

}  // namespace custom_controller

#endif  // CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_