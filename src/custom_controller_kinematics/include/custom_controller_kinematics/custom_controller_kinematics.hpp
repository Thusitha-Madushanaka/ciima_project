#ifndef CUSTOM_CONTROLLER_KINEMATICS_HPP_
#define CUSTOM_CONTROLLER_KINEMATICS_HPP_

namespace custom_controller_kinematics
{

// define the robot velocity(linear.x, linear.y, angular velocity)
struct RobotVelocity
{
    double vx;
    double vy;
    double omega;
};

// deifne the wheel Geometry of Omini 1, and Omni 2
struct WheelGeometry
{
    double x;
    double y;
    double radius;
};

// initliaze the wheel velocity and wheel angle
struct WheelCommand
{
    double steering_angle;
    double wheel_velocity;
};

struct KinematicsResult
{
    WheelCommand omni1;
    WheelCommand omni2;
};


class CustomControllerKinematics
{
public:

    CustomControllerKinematics(
        const WheelGeometry& omni1,
        const WheelGeometry& omni2
    );

    // deifne the inverse kinematics (vx, vy, omega convert into wheel velocity and angle)

    KinematicsResult inverseKinematics(
        const RobotVelocity& velocity);
    
        //define the forward kinematics (wheel commands to vx, vy and omega)

    RobotVelocity forwardKinematics(
        const KinematicsResult& wheel_commands);
    

private:

    WheelGeometry omni1_;
    WheelGeometry omni2_;
};

}  // namespace custom_controller_kinematics

#endif  // CUSTOM_CONTROLLER_KINEMATICS_HPP_