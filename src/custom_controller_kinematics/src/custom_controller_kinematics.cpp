#include <cmath>
#include "custom_controller_kinematics/custom_controller_kinematics.hpp"


namespace custom_controller_kinematics
{

CustomControllerKinematics::CustomControllerKinematics(
    const WheelGeometry& omni1,
    const WheelGeometry& omni2)
    : omni1_(omni1),
      omni2_(omni2)
{
}


KinematicsResult CustomControllerKinematics::inverseKinematics(
    const RobotVelocity& velocity)

{
    KinematicsResult result;
    
    // OmniDrive 1
    // calculate the omni1 velocity

    double wheel_v1x = velocity.vx - velocity.omega * omni1_.y;
    double wheel_v1y = velocity.vy + velocity.omega * omni1_.x;

    // calculate the speed of the omni 1

    double speed1 = std::sqrt(wheel_v1x * wheel_v1x + wheel_v1y * wheel_v1y);

    // calculate the steering angle

    if (speed1 > 1e-9)
    {
    result.omni1.steering_angle =
        std::atan2(wheel_v1y, wheel_v1x);
    }
    else
    {
    result.omni1.steering_angle = 0.0;
    }

    

    // calculate the wheel velocity

    result.omni1.wheel_velocity = speed1 / omni1_.radius;


    // OmniDrive 2

    // calculate the omni2 velocity

    double wheel_v2x = velocity.vx - velocity.omega * omni2_.y;
    double wheel_v2y = velocity.vy + velocity.omega * omni2_.x;

    // calculate the speed of omni 2

    double speed2 = std ::sqrt(wheel_v2x * wheel_v2x + wheel_v2y * wheel_v2y);

    // calculate the steering angle

    if (speed2 > 1e-9)
    {
    result.omni2.steering_angle = std::atan2(wheel_v2y, wheel_v2x);
    }
    else
    {
    result.omni2.steering_angle = 0.0;
    }

    

    // calculate the wheel velocity

    result.omni2.wheel_velocity = speed2 / omni2_.radius;


    return result;

}

RobotVelocity CustomControllerKinematics::forwardKinematics(
    const KinematicsResult& wheel_commands)
{
    RobotVelocity velocity{};

    double speed1 =
        omni1_.radius * wheel_commands.omni1.wheel_velocity;

    double speed2 =
        omni2_.radius * wheel_commands.omni2.wheel_velocity;

    double wheel_v1x =
        speed1 *
        std::cos(wheel_commands.omni1.steering_angle);

    double wheel_v1y =
        speed1 *
        std::sin(wheel_commands.omni1.steering_angle);

    double wheel_v2x =
        speed2 *
        std::cos(wheel_commands.omni2.steering_angle);

    double wheel_v2y =
        speed2 *
        std::sin(wheel_commands.omni2.steering_angle);

    velocity.omega =
        (wheel_v1y - wheel_v2y) /
        (omni1_.x - omni2_.x);

    velocity.vx =
        (wheel_v1x + wheel_v2x) / 2.0;

    velocity.vy =
    (wheel_v1y + wheel_v2y) / 2.0;

    return velocity;
}
}