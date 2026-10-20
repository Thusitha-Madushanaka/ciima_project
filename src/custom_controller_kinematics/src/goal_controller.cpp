#include "custom_controller_kinematics/goal_controller.hpp"

#include <cmath>

namespace custom_controller_kinematics
{

GoalController::GoalController(
  const GoalControllerParameters & parameters)
: parameters_(parameters)
{
}

double GoalController::normalizeAngle(double angle) const
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }

  return angle;
}

VelocityCommand GoalController::computeCommand(
  const RobotPose & current_pose,
  const GoalPose & goal_pose) const
{
  // Calculate position error in the world/odom frame
  double error_x_world =
    goal_pose.x - current_pose.x;

  double error_y_world =
    goal_pose.y - current_pose.y;

  // Transform position error from world frame to robot frame
  double error_x_robot =
    std::cos(current_pose.yaw) * error_x_world +
    std::sin(current_pose.yaw) * error_y_world;

  double error_y_robot =
    -std::sin(current_pose.yaw) * error_x_world +
    std::cos(current_pose.yaw) * error_y_world;

  // Calculate orientation error
  double error_yaw =
    normalizeAngle(goal_pose.yaw - current_pose.yaw);

  // Calculate distance to the goal
  double position_error =
    std::sqrt(
      error_x_world * error_x_world +
      error_y_world * error_y_world);

  // Initialize velocity command
  VelocityCommand command{};

  // Check whether the goal has been reached
  if (position_error <= parameters_.position_tolerance &&
      std::abs(error_yaw) <= parameters_.yaw_tolerance)
  {
    return command;
  }

  // Proportional control for linear velocity
  command.vx =
    parameters_.kp_x * error_x_robot;

  command.vy =
    parameters_.kp_y * error_y_robot;

  // Proportional control for angular velocity
  command.omega =
    parameters_.kp_yaw * error_yaw;

  // Limit linear velocity in x
  if (command.vx > parameters_.max_vx)
  {
    command.vx = parameters_.max_vx;
  }
  else if (command.vx < -parameters_.max_vx)
  {
    command.vx = -parameters_.max_vx;
  }

  // Limit linear velocity in y
  if (command.vy > parameters_.max_vy)
  {
    command.vy = parameters_.max_vy;
  }
  else if (command.vy < -parameters_.max_vy)
  {
    command.vy = -parameters_.max_vy;
  }

  // Limit angular velocity
  if (command.omega > parameters_.max_omega)
  {
    command.omega = parameters_.max_omega;
  }
  else if (command.omega < -parameters_.max_omega)
  {
    command.omega = -parameters_.max_omega;
  }

  return command;
}

}  // namespace custom_controller_kinematics

