#include <iostream>
#include <cmath>

#include "custom_controller_kinematics/custom_controller_kinematics.hpp"
#include "custom_controller_kinematics/goal_controller.hpp"

using namespace custom_controller_kinematics;


// ============================================================
// Helper function for comparing floating-point values
// ============================================================

bool approximatelyEqual(
    double value,
    double expected,
    double tolerance = 1e-5)
{
    return std::abs(value - expected) <= tolerance;
}


// ============================================================
// Kinematics test function
// ============================================================

bool test_velocity(
    const char * test_name,
    const RobotVelocity & robot_velocity,
    CustomControllerKinematics & kinematics)
{
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << test_name << "\n";
    std::cout << "========================================\n";

    // Inverse kinematics
    auto wheel_commands =
        kinematics.inverseKinematics(robot_velocity);

    std::cout << "Inverse kinematics:\n";

    std::cout << "  Omni 1:\n";
    std::cout << "    steering angle = "
              << wheel_commands.omni1.steering_angle
              << " rad\n";

    std::cout << "    wheel velocity = "
              << wheel_commands.omni1.wheel_velocity
              << " rad/s\n";

    std::cout << "  Omni 2:\n";
    std::cout << "    steering angle = "
              << wheel_commands.omni2.steering_angle
              << " rad\n";

    std::cout << "    wheel velocity = "
              << wheel_commands.omni2.wheel_velocity
              << " rad/s\n";


    // Forward kinematics
    RobotVelocity recovered_velocity =
        kinematics.forwardKinematics(wheel_commands);

    std::cout << "\nForward kinematics:\n";

    std::cout << "  vx = "
              << recovered_velocity.vx
              << " m/s\n";

    std::cout << "  vy = "
              << recovered_velocity.vy
              << " m/s\n";

    std::cout << "  omega = "
              << recovered_velocity.omega
              << " rad/s\n";


    // Check result
    bool passed =
        approximatelyEqual(
            recovered_velocity.vx,
            robot_velocity.vx) &&
        approximatelyEqual(
            recovered_velocity.vy,
            robot_velocity.vy) &&
        approximatelyEqual(
            recovered_velocity.omega,
            robot_velocity.omega);


    if (passed)
    {
        std::cout << "\nRESULT: PASS\n";
    }
    else
    {
        std::cout << "\nRESULT: FAIL\n";
    }

    return passed;
}


// ============================================================
// Main
// ============================================================

int main()
{
    // ========================================================
    // Robot geometry
    // ========================================================

    WheelGeometry omni1;

    omni1.x = 0.36;
    omni1.y = 0.0;
    omni1.radius = 0.10;


    WheelGeometry omni2;

    omni2.x = -0.36;
    omni2.y = 0.0;
    omni2.radius = 0.10;


    CustomControllerKinematics kinematics(
        omni1,
        omni2);


    // ========================================================
    // KINEMATICS TEST 1: FORWARD MOTION
    // ========================================================

    RobotVelocity forward;

    forward.vx = 0.5;
    forward.vy = 0.0;
    forward.omega = 0.0;

    bool test1 =
        test_velocity(
            "TEST 1: FORWARD MOTION",
            forward,
            kinematics);


    // ========================================================
    // KINEMATICS TEST 2: SIDEWAYS MOTION
    // ========================================================

    RobotVelocity sideways;

    sideways.vx = 0.0;
    sideways.vy = 0.5;
    sideways.omega = 0.0;

    bool test2 =
        test_velocity(
            "TEST 2: SIDEWAYS MOTION",
            sideways,
            kinematics);


    // ========================================================
    // KINEMATICS TEST 3: ROTATION
    // ========================================================

    RobotVelocity rotation;

    rotation.vx = 0.0;
    rotation.vy = 0.0;
    rotation.omega = 0.5;

    bool test3 =
        test_velocity(
            "TEST 3: ROTATION",
            rotation,
            kinematics);


    // ========================================================
    // KINEMATICS TEST 4: COMBINED MOTION
    // ========================================================

    RobotVelocity combined;

    combined.vx = 0.5;
    combined.vy = 0.2;
    combined.omega = 0.3;

    bool test4 =
        test_velocity(
            "TEST 4: COMBINED MOTION",
            combined,
            kinematics);


    // ========================================================
    // Goal Controller Parameters
    // ========================================================

    GoalControllerParameters parameters;

    parameters.kp_x = 0.5;
    parameters.kp_y = 0.5;
    parameters.kp_yaw = 1.0;

    parameters.max_vx = 1.0;
    parameters.max_vy = 1.0;
    parameters.max_omega = 1.0;

    parameters.position_tolerance = 0.02;
    parameters.yaw_tolerance = 0.05;


    GoalController goal_controller(parameters);


    // ========================================================
    // GOAL CONTROLLER TEST 1: MOVE FORWARD
    // ========================================================

    RobotPose current_pose;

    GoalPose goal_pose;

    current_pose.x = 0.0;
    current_pose.y = 0.0;
    current_pose.yaw = 0.0;

    goal_pose.x = 1.0;
    goal_pose.y = 0.0;
    goal_pose.yaw = 0.0;


    VelocityCommand goal_command =
        goal_controller.computeCommand(
            current_pose,
            goal_pose);


    bool goal_test1 =
        approximatelyEqual(
            goal_command.vx,
            0.5) &&
        approximatelyEqual(
            goal_command.vy,
            0.0) &&
        approximatelyEqual(
            goal_command.omega,
            0.0);


    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "GOAL CONTROLLER TEST 1: MOVE FORWARD\n";
    std::cout << "========================================\n";

    std::cout << "Current pose:\n";
    std::cout << "  x   = " << current_pose.x << " m\n";
    std::cout << "  y   = " << current_pose.y << " m\n";
    std::cout << "  yaw = " << current_pose.yaw << " rad\n";

    std::cout << "\nGoal pose:\n";
    std::cout << "  x   = " << goal_pose.x << " m\n";
    std::cout << "  y   = " << goal_pose.y << " m\n";
    std::cout << "  yaw = " << goal_pose.yaw << " rad\n";

    std::cout << "\nVelocity command:\n";
    std::cout << "  vx    = " << goal_command.vx << " m/s\n";
    std::cout << "  vy    = " << goal_command.vy << " m/s\n";
    std::cout << "  omega = " << goal_command.omega << " rad/s\n";

    std::cout << "\nRESULT: "
              << (goal_test1 ? "PASS" : "FAIL")
              << "\n";


    // ========================================================
    // GOAL CONTROLLER TEST 2: MOVE SIDEWAYS
    // ========================================================

    current_pose.x = 0.0;
    current_pose.y = 0.0;
    current_pose.yaw = 0.0;

    goal_pose.x = 0.0;
    goal_pose.y = 1.0;
    goal_pose.yaw = 0.0;


    goal_command =
        goal_controller.computeCommand(
            current_pose,
            goal_pose);


    bool goal_test2 =
        approximatelyEqual(
            goal_command.vx,
            0.0) &&
        approximatelyEqual(
            goal_command.vy,
            0.5) &&
        approximatelyEqual(
            goal_command.omega,
            0.0);


    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "GOAL CONTROLLER TEST 2: MOVE SIDEWAYS\n";
    std::cout << "========================================\n";

    std::cout << "Velocity command:\n";
    std::cout << "  vx    = " << goal_command.vx << " m/s\n";
    std::cout << "  vy    = " << goal_command.vy << " m/s\n";
    std::cout << "  omega = " << goal_command.omega << " rad/s\n";

    std::cout << "\nRESULT: "
              << (goal_test2 ? "PASS" : "FAIL")
              << "\n";


    // ========================================================
    // GOAL CONTROLLER TEST 3: ROTATION
    // ========================================================

    current_pose.x = 0.0;
    current_pose.y = 0.0;
    current_pose.yaw = 0.0;

    goal_pose.x = 0.0;
    goal_pose.y = 0.0;
    goal_pose.yaw = M_PI / 2.0;


    goal_command =
        goal_controller.computeCommand(
            current_pose,
            goal_pose);


    bool goal_test3 =
        approximatelyEqual(
            goal_command.vx,
            0.0) &&
        approximatelyEqual(
            goal_command.vy,
            0.0) &&
        approximatelyEqual(
            goal_command.omega,
            1.0);


    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "GOAL CONTROLLER TEST 3: ROTATION\n";
    std::cout << "========================================\n";

    std::cout << "Velocity command:\n";
    std::cout << "  vx    = " << goal_command.vx << " m/s\n";
    std::cout << "  vy    = " << goal_command.vy << " m/s\n";
    std::cout << "  omega = " << goal_command.omega << " rad/s\n";

    std::cout << "\nRESULT: "
              << (goal_test3 ? "PASS" : "FAIL")
              << "\n";


    // ========================================================
    // GOAL CONTROLLER TEST 4: WITHIN TOLERANCE
    // ========================================================

    current_pose.x = 0.0;
    current_pose.y = 0.0;
    current_pose.yaw = 0.0;

    goal_pose.x = 0.01;
    goal_pose.y = 0.0;
    goal_pose.yaw = 0.0;


    goal_command =
        goal_controller.computeCommand(
            current_pose,
            goal_pose);


    bool goal_test4 =
        approximatelyEqual(
            goal_command.vx,
            0.0) &&
        approximatelyEqual(
            goal_command.vy,
            0.0) &&
        approximatelyEqual(
            goal_command.omega,
            0.0);


    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "GOAL CONTROLLER TEST 4: WITHIN TOLERANCE\n";
    std::cout << "========================================\n";

    std::cout << "Distance to goal = 0.01 m\n";

    std::cout << "\nVelocity command:\n";
    std::cout << "  vx    = " << goal_command.vx << " m/s\n";
    std::cout << "  vy    = " << goal_command.vy << " m/s\n";
    std::cout << "  omega = " << goal_command.omega << " rad/s\n";

    std::cout << "\nRESULT: "
              << (goal_test4 ? "PASS" : "FAIL")
              << "\n";


    // ========================================================
    // GOAL CONTROLLER TEST 5: ROTATED ROBOT
    // ========================================================

    current_pose.x = 0.0;
    current_pose.y = 0.0;
    current_pose.yaw = M_PI / 2.0;

    goal_pose.x = 0.0;
    goal_pose.y = 1.0;
    goal_pose.yaw = M_PI / 2.0;


    goal_command =
        goal_controller.computeCommand(
            current_pose,
            goal_pose);


    bool goal_test5 =
        approximatelyEqual(
            goal_command.vx,
            0.5) &&
        approximatelyEqual(
            goal_command.vy,
            0.0) &&
        approximatelyEqual(
            goal_command.omega,
            0.0);


    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "GOAL CONTROLLER TEST 5: ROTATED ROBOT\n";
    std::cout << "========================================\n";

    std::cout << "Current pose:\n";
    std::cout << "  x   = " << current_pose.x << " m\n";
    std::cout << "  y   = " << current_pose.y << " m\n";
    std::cout << "  yaw = " << current_pose.yaw << " rad\n";

    std::cout << "\nGoal pose:\n";
    std::cout << "  x   = " << goal_pose.x << " m\n";
    std::cout << "  y   = " << goal_pose.y << " m\n";
    std::cout << "  yaw = " << goal_pose.yaw << " rad\n";

    std::cout << "\nVelocity command:\n";
    std::cout << "  vx    = " << goal_command.vx << " m/s\n";
    std::cout << "  vy    = " << goal_command.vy << " m/s\n";
    std::cout << "  omega = " << goal_command.omega << " rad/s\n";

    std::cout << "\nRESULT: "
              << (goal_test5 ? "PASS" : "FAIL")
              << "\n";


    // ========================================================
    // Overall result
    // ========================================================

    bool all_tests_passed =
        test1 &&
        test2 &&
        test3 &&
        test4 &&
        goal_test1 &&
        goal_test2 &&
        goal_test3 &&
        goal_test4 &&
        goal_test5;


    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << " OVERALL RESULT\n";
    std::cout << "========================================\n";

    if (all_tests_passed)
    {
        std::cout << "ALL TESTS PASSED\n";
    }
    else
    {
        std::cout << "SOME TESTS FAILED\n";
    }

    std::cout << "\n";

    return all_tests_passed ? 0 : 1;
}