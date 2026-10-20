#include "custom_controller/custom_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <cmath>

using namespace std;


namespace custom_controller
{


// CONSTRUCTOR


CustomController::CustomController()
{
  custom_controller_kinematics::WheelGeometry omni1;

  omni1.x = 0.36;
  omni1.y = 0.0;
  omni1.radius = 0.0725;


  custom_controller_kinematics::WheelGeometry omni2;

  omni2.x = -0.36;
  omni2.y = 0.0;
  omni2.radius = 0.0725;


  kinematics_ =
    std::make_unique<
      custom_controller_kinematics::CustomControllerKinematics>(
        omni1,
        omni2
      );
}



// ON INIT


controller_interface::CallbackReturn
CustomController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}



// COMMAND INTERFACE CONFIGURATION


controller_interface::InterfaceConfiguration
CustomController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;

  config.type =
    controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names = {
    "fake_robot_front_steer_joint/position",
    "fake_robot_rear_steer_joint/position",
    "fake_robot_front_wheel_joint/velocity",
    "fake_robot_rear_wheel_joint/velocity"
  };

  return config;
}



// STATE INTERFACE CONFIGURATION


controller_interface::InterfaceConfiguration
CustomController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;

  config.type =
    controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names = {

    // Front steering
    "fake_robot_front_steer_joint/position",
    "fake_robot_front_steer_joint/velocity",

    // Rear steering
    "fake_robot_rear_steer_joint/position",
    "fake_robot_rear_steer_joint/velocity",

    // Front wheel
    "fake_robot_front_wheel_joint/position",
    "fake_robot_front_wheel_joint/velocity",

    // Rear wheel
    "fake_robot_rear_wheel_joint/position",
    "fake_robot_rear_wheel_joint/velocity"
  };

  return config;
}



// ON CONFIGURE


controller_interface::CallbackReturn
CustomController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  // Read parameters
  

  get_node()->get_parameter(
    "cmd_vel_timeout",
    cmd_vel_timeout_);


  get_node()->get_parameter(
    "max_steering_rate",
    max_steering_rate_);


  // This parameter is optional.
  //
  // If it does not exist in YAML, the default value
  // from the header remains 0.05 rad.

  if (get_node()->has_parameter("steering_alignment_tolerance"))
  {
    get_node()->get_parameter(
      "steering_alignment_tolerance",
      steering_alignment_tolerance_);
  }


  
  // /cmd_vel subscriber
  

  cmd_vel_subscriber_ =
    get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        latest_cmd_vel_ = msg;

        last_cmd_time_ =
          get_node()->get_clock()->now();


        RCLCPP_INFO_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(),
          2000,
          "cmd_vel: vx=%.2f, vy=%.2f, omega=%.2f",
          msg->linear.x,
          msg->linear.y,
          msg->angular.z
        );
      });


  
  // TF broadcaster
  

  tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(
      get_node());


  
  // Odometry publisher
  

  odom_publisher_ =
    get_node()->create_publisher<nav_msgs::msg::Odometry>(
      "/odom",
      10);


  return controller_interface::CallbackReturn::SUCCESS;
}



// ON ACTIVATE


controller_interface::CallbackReturn
CustomController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize the internal steering commands
  // from the actual steering joint positions.

  if (state_interfaces_.size() >= 8)
  {
    commanded_steering_angle_1_ =
      state_interfaces_[0].get_value();

    commanded_steering_angle_2_ =
      state_interfaces_[2].get_value();
  }
  else
  {
    commanded_steering_angle_1_ = 0.0;
    commanded_steering_angle_2_ = 0.0;
  }


  return controller_interface::CallbackReturn::SUCCESS;
}



// ON DEACTIVATE


controller_interface::CallbackReturn
CustomController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}



// UPDATE


controller_interface::return_type
CustomController::update(
  const rclcpp::Time & current_time,
  const rclcpp::Duration & period)
{
  (void)current_time;


  double dt = period.seconds();


 
  // ODOMETRY
  

  custom_controller_kinematics::KinematicsResult feedback;


  
  // Read actual steering angles
  

  feedback.omni1.steering_angle =
    state_interfaces_[0].get_value();

  feedback.omni2.steering_angle =
    state_interfaces_[2].get_value();


  
  // Read actual wheel velocities
  

  feedback.omni1.wheel_velocity =
    state_interfaces_[5].get_value();

  feedback.omni2.wheel_velocity =
    state_interfaces_[7].get_value();


  
  // Forward kinematics
  

  custom_controller_kinematics::RobotVelocity robot_velocity =
    kinematics_->forwardKinematics(feedback);


  
  // Convert robot-frame velocity to odom frame
  

  double vx_odom =
    robot_velocity.vx * std::cos(yaw_) -
    robot_velocity.vy * std::sin(yaw_);


  double vy_odom =
    robot_velocity.vx * std::sin(yaw_) +
    robot_velocity.vy * std::cos(yaw_);


  
  // Integrate odometry
  

  x_ = x_ + vx_odom * dt;

  y_ = y_ + vy_odom * dt;

  yaw_ = yaw_ + robot_velocity.omega * dt;


  
  // ODOM MESSAGE
  

  nav_msgs::msg::Odometry odom;


  odom.header.stamp = get_node()->get_clock()->now();

  odom.header.frame_id = "odom";

  odom.child_frame_id = "fake_robot_base_link";


  
  // Position
  

  odom.pose.pose.position.x = x_;

  odom.pose.pose.position.y = y_;

  odom.pose.pose.position.z = 0.0;


  
  // Orientation
  

  odom.pose.pose.orientation.x = 0.0;

  odom.pose.pose.orientation.y = 0.0;

  odom.pose.pose.orientation.z = std::sin(yaw_ / 2.0);

  odom.pose.pose.orientation.w = std::cos(yaw_ / 2.0);


  
  // Velocity
  

  odom.twist.twist.linear.x = robot_velocity.vx;

  odom.twist.twist.linear.y = robot_velocity.vy;

  odom.twist.twist.linear.z = 0.0;

  odom.twist.twist.angular.x = 0.0;

  odom.twist.twist.angular.y = 0.0;

  odom.twist.twist.angular.z = robot_velocity.omega;


  odom_publisher_->publish(odom);


  
  // TF
  

  geometry_msgs::msg::TransformStamped transform;


  transform.header.stamp = get_node()->get_clock()->now();

  transform.header.frame_id = "odom";

  transform.child_frame_id = "fake_robot_base_link";


  transform.transform.translation.x = x_;

  transform.transform.translation.y = y_;

  transform.transform.translation.z = 0.0;


  transform.transform.rotation.x = 0.0;

  transform.transform.rotation.y = 0.0;

  transform.transform.rotation.z = std::sin(yaw_ / 2.0);

  transform.transform.rotation.w = std::cos(yaw_ / 2.0);


  tf_broadcaster_->sendTransform(transform);


  
  // DEBUG ODOMETRY
  

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    1000,
    "Odometry pose: x=%.3f m, y=%.3f m, yaw=%.3f rad",
    x_,
    y_,
    yaw_);


  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    1000,
    "Odometry velocity: vx=%.3f m/s, vy=%.3f m/s, omega=%.3f rad/s",
    robot_velocity.vx,
    robot_velocity.vy,
    robot_velocity.omega);


  
  // CMD_VEL AND TIMEOUT
  

  if (latest_cmd_vel_ && kinematics_)
  {
    double time_since_last_cmd =
      (get_node()->get_clock()->now() - last_cmd_time_).seconds();


    custom_controller_kinematics::RobotVelocity command_velocity;


    
    // Check command timeout
    

    if (time_since_last_cmd > cmd_vel_timeout_)
    {
      command_velocity.vx = 0.0;
      command_velocity.vy = 0.0;
      command_velocity.omega = 0.0;


      RCLCPP_INFO_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "cmd_vel timeout: stopping robot");
    }
    else
    {
      command_velocity.vx =
        latest_cmd_vel_->linear.x;

      command_velocity.vy =
        latest_cmd_vel_->linear.y;

      command_velocity.omega =
        latest_cmd_vel_->angular.z;
    }


    
    // INVERSE KINEMATICS
    

    auto wheel_commands =
      kinematics_->inverseKinematics(command_velocity);


    RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Desired Omni1: angle=%.2f rad, velocity=%.2f rad/s | "
      "Desired Omni2: angle=%.2f rad, velocity=%.2f rad/s",
      wheel_commands.omni1.steering_angle,
      wheel_commands.omni1.wheel_velocity,
      wheel_commands.omni2.steering_angle,
      wheel_commands.omni2.wheel_velocity);


    
    // STEERING RATE LIMITING
    

    double max_steering_change = max_steering_rate_ * dt;


    
    // Omni 1
    

    double steering_error_1 =
      std::atan2(
        std::sin(
          wheel_commands.omni1.steering_angle -
          commanded_steering_angle_1_),
        std::cos(
          wheel_commands.omni1.steering_angle -
          commanded_steering_angle_1_)
      );


    if (steering_error_1 > max_steering_change)
    {
      steering_error_1 =
        max_steering_change;
    }
    else if (steering_error_1 < -max_steering_change)
    {
      steering_error_1 =
        -max_steering_change;
    }


    commanded_steering_angle_1_ +=
      steering_error_1;


    
    // Omni 2
    

    double steering_error_2 =
      std::atan2(
        std::sin(
          wheel_commands.omni2.steering_angle -
          commanded_steering_angle_2_),
        std::cos(
          wheel_commands.omni2.steering_angle -
          commanded_steering_angle_2_)
      );


    if (steering_error_2 > max_steering_change)
    {
      steering_error_2 =
        max_steering_change;
    }
    else if (steering_error_2 < -max_steering_change)
    {
      steering_error_2 =
        -max_steering_change;
    }


    commanded_steering_angle_2_ +=
      steering_error_2;


    
    // ACTUAL STEERING ALIGNMENT CHECK
    

    // Read the REAL steering joint positions.
    //
    // This is important.
    //
    // commanded_steering_angle_1_ is what we ask the joint to do.
    //
    // state_interfaces_[0] is what the joint actually reports.

    double actual_steering_1 =
      state_interfaces_[0].get_value();

    double actual_steering_2 =
      state_interfaces_[2].get_value();


    
    // Calculate shortest angular error
    

    double alignment_error_1 =
      std::atan2(
        std::sin(
          wheel_commands.omni1.steering_angle -
          actual_steering_1),
        std::cos(
          wheel_commands.omni1.steering_angle -
          actual_steering_1)
      );


    double alignment_error_2 =
      std::atan2(
        std::sin(
          wheel_commands.omni2.steering_angle -
          actual_steering_2),
        std::cos(
          wheel_commands.omni2.steering_angle -
          actual_steering_2)
      );


    
    // Check whether steering is aligned
    

    bool steering_aligned =
      std::abs(alignment_error_1) <=
        steering_alignment_tolerance_
      &&
      std::abs(alignment_error_2) <=
        steering_alignment_tolerance_;


    
    // WRITE STEERING COMMANDS
    

    command_interfaces_[0].set_value(
      commanded_steering_angle_1_);


    command_interfaces_[1].set_value(
      commanded_steering_angle_2_);

    RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Steering commands written: front=%.3f, rear=%.3f",
      commanded_steering_angle_1_,
      commanded_steering_angle_2_);


    
    // WRITE WHEEL COMMANDS
    

    if (steering_aligned)
    {
      // ------------------------------------------------------
      // Steering is correctly oriented.
      //
      // The wheel motors are now allowed to move.
      // ------------------------------------------------------

      command_interfaces_[2].set_value(
        wheel_commands.omni1.wheel_velocity);


      command_interfaces_[3].set_value(
        wheel_commands.omni2.wheel_velocity);


      RCLCPP_INFO_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Steering aligned -> wheel motion enabled");
    }
    else
    {
      
      // Steering is still moving.
      //
      // Keep the wheels stopped.
      

      command_interfaces_[2].set_value(0.0);
      command_interfaces_[3].set_value(0.0);


      RCLCPP_INFO_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Steering aligning -> wheels stopped "
        "(error1=%.3f, error2=%.3f)",
        alignment_error_1,
        alignment_error_2);
    }
  }


  return controller_interface::return_type::OK;
}

}  // namespace custom_controller


// ============================================================
// PLUGIN EXPORT
// ============================================================

PLUGINLIB_EXPORT_CLASS(
  custom_controller::CustomController,
  controller_interface::ControllerInterface
)

