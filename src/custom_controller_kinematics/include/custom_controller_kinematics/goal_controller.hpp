#ifndef CUSTOM_CONTROLLER_KINEMATICS__GOAL_CONTROLLER_HPP_
#define CUSTOM_CONTROLLER_KINEMATICS__GOAL_CONTROLLER_HPP_

using namespace std;

namespace custom_controller_kinematics
{

struct RobotPose
{
  double x;
  double y;
  double yaw;
};

struct GoalPose
{
  double x;
  double y;
  double yaw;
};

struct GoalControllerParameters
{
  double kp_x;
  double kp_y;
  double kp_yaw;

  double max_vx;
  double max_vy;
  double max_omega;

  // to check the where the robot is reached to the goal or not

  double position_tolerance;
  double yaw_tolerance;
};

struct VelocityCommand
{
  double vx;
  double vy;
  double omega;
};

class GoalController
{
public:

  explicit GoalController(
    const GoalControllerParameters & parameters);

  VelocityCommand computeCommand(
    const RobotPose & current_pose,
    const GoalPose & goal_pose) const;

private:

  GoalControllerParameters parameters_;

  double normalizeAngle(double angle) const;
};

}  // namespace custom_controller_kinematics

#endif  // CUSTOM_CONTROLLER_KINEMATICS__GOAL_CONTROLLER_HPP_