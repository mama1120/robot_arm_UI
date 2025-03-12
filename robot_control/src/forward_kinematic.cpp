#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>

// Function to move the robot arm to a specific joint configuration
void move_to_joint_positions(moveit::planning_interface::MoveGroupInterface& move_group, 
                             const std::vector<double>& joint_positions,
                             const std::string& logger_name) {
    // Set the target joint positions
    move_group.setJointValueTarget(joint_positions);

    // Plan and execute the movement
    bool success = static_cast<bool>(move_group.move());
    if (success) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Movement to joint positions successful");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Movement to joint positions failed");
    }
}

// Function to control the gripper (plate)
void control_gripper(moveit::planning_interface::MoveGroupInterface& move_group, 
                     const std::vector<double>& gripper_position,
                     const std::string& logger_name) {
    // Set the target joint positions for the gripper
    move_group.setJointValueTarget(gripper_position);

    // Plan and execute the movement
    bool success = static_cast<bool>(move_group.move());
    if (success) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Gripper movement successful");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Gripper movement failed");
    }
}

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "joint_position_setter", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create the MoveIt MoveGroup Interface for the robot arm
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_robot_arm = std::make_shared<MoveGroupInterface>(node, "robot_arm");
    move_group_robot_arm->setPlannerId("RRTConnectConfigDefault");
    move_group_robot_arm->setPlanningTime(100.0);
    move_group_robot_arm->setNumPlanningAttempts(10);

    // Create the MoveIt MoveGroup Interface for the hand (gripper)
    auto move_group_hand = std::make_shared<MoveGroupInterface>(node, "hand");

    // Define the target joint positions for the robot arm
    std::vector<double> target_joint_positions = {
        -0.20976713960918494,  // joint2
        -0.45866962529116184,  // joint3
        -0.5101330854729407,   // joint1
        0.8688616648080048     // joint4
    };

    // Define the target joint position for the gripper (plate)
    std::vector<double> target_gripper_position = {0.0};  // Example: 0.0 for closed, 1.0 for open

    // Move the robot arm to the target joint positions
    RCLCPP_INFO(node->get_logger(), "Moving robot arm to target joint positions...");
    move_to_joint_positions(*move_group_robot_arm, target_joint_positions, "joint_position_setter");

    // Control the gripper (plate)
    RCLCPP_INFO(node->get_logger(), "Controlling gripper...");
    control_gripper(*move_group_hand, target_gripper_position, "joint_position_setter");

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}