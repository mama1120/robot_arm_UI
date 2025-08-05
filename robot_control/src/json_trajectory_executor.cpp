/**
 * This program loads a sequence of joint states from a JSON file and executes them sequentially
 * on the robot arm with gripper control. The json file can be created with the UI tool in 
 * rviz. Simply create a sequence and save it as a JSON file.
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Structure to hold joint state information
struct JointState {
    std::string name;
    double joint1;
    double joint2;
    double joint3;
    double joint4;
    double gripper;
};

// Function to move the robot arm to a specific joint configuration
void move_to_joint_positions(moveit::planning_interface::MoveGroupInterface& move_group, 
                             const std::vector<double>& joint_positions,
                             const std::string& logger_name,
                             const std::string& position_name) {
    // Set the target joint positions
    move_group.setJointValueTarget(joint_positions);

    // Plan and execute the movement
    bool success = static_cast<bool>(move_group.move());
    if (success) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Movement to position '%s' successful", position_name.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Movement to position '%s' failed", position_name.c_str());
    }
}

// Function to control the gripper (plate)
void control_gripper(moveit::planning_interface::MoveGroupInterface& move_group, 
                     const std::vector<double>& gripper_position,
                     const std::string& logger_name,
                     const std::string& position_name) {
    
    RCLCPP_INFO(rclcpp::get_logger(logger_name), "Setting gripper to position: %.6f for '%s'", 
                gripper_position[0], position_name.c_str());
    
    // The gripper has 4 joints: joint_finger1, joint_finger2, joint_finger3, joint_plate
    // Set all gripper joints to the same value (assuming symmetric gripper)
    std::vector<double> all_gripper_positions = {
        gripper_position[0],  // joint_finger1
        gripper_position[0],  // joint_finger2  
        gripper_position[0],  // joint_finger3
        gripper_position[0]   // joint_plate
    };
    
    // Set the target joint positions for all gripper joints
    move_group.setJointValueTarget(all_gripper_positions);

    // Plan and execute the movement
    bool success = static_cast<bool>(move_group.move());
    if (success) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Gripper movement for position '%s' successful", position_name.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Gripper movement for position '%s' failed", position_name.c_str());
    }
}

// Function to load joint states from JSON file
std::vector<JointState> load_joint_states_from_json(const std::string& file_path) {
    std::vector<JointState> joint_states;
    
    try {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file: " + file_path);
        }

        json j;
        file >> j;

        for (const auto& point : j["points"]) {
            JointState state;
            state.name = point["name"];
            state.joint1 = point["joint1"];
            state.joint2 = point["joint2"];
            state.joint3 = point["joint3"];
            state.joint4 = point["joint4"];
            state.gripper = point["gripper"];
            joint_states.push_back(state);
        }

        RCLCPP_INFO(rclcpp::get_logger("json_loader"), "Successfully loaded %zu joint states from %s", 
                    joint_states.size(), file_path.c_str());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("json_loader"), "Error loading JSON file: %s", e.what());
        throw;
    }

    return joint_states;
}

// Function to execute trajectory from joint states
void execute_trajectory(moveit::planning_interface::MoveGroupInterface& move_group_robot_arm,
                       moveit::planning_interface::MoveGroupInterface& move_group_hand,
                       const std::vector<JointState>& joint_states,
                       const std::string& logger_name) {
    
    for (size_t i = 0; i < joint_states.size(); ++i) {
        const auto& state = joint_states[i];
        
        RCLCPP_INFO(rclcpp::get_logger(logger_name), 
                    "Moving to position %zu/%zu: '%s'", i + 1, joint_states.size(), state.name.c_str());

        // Prepare joint positions for robot arm (assuming the order is joint1, joint2, joint3, joint4)
        std::vector<double> arm_joint_positions = {
            state.joint1,
            state.joint2,
            state.joint3,
            state.joint4
        };

        // Prepare gripper position
        std::vector<double> gripper_position = {state.gripper};

        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Position '%s': arm joints=[%.3f, %.3f, %.3f, %.3f], gripper=%.6f", 
                    state.name.c_str(), state.joint1, state.joint2, state.joint3, state.joint4, state.gripper);

        // Move robot arm to target position
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Moving robot arm to position '%s'...", state.name.c_str());
        move_to_joint_positions(move_group_robot_arm, arm_joint_positions, logger_name, state.name);

        // Control gripper
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Controlling gripper for position '%s'...", state.name.c_str());
        control_gripper(move_group_hand, gripper_position, logger_name, state.name);

        // TODO: Add camera picture taking function here in the future
        // take_picture(state.name);

        // Wait 5 seconds before moving to next position (except for the last position)
        if (i < joint_states.size() - 1) {
            RCLCPP_INFO(rclcpp::get_logger(logger_name), "Waiting 5 seconds before next movement... (Insert camera functionality here)");
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }

    RCLCPP_INFO(rclcpp::get_logger(logger_name), "Trajectory execution completed!");
}

int main(int argc, char* argv[]) {
    // Check if JSON file path is provided as argument
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <json_file_path>" << std::endl;
        std::cerr <<"Json file path required as an argument!" << std::endl;
        std::cerr <<"Example: ros2 run robot_control json_trajectory_executor /home/user/robot_ws/src/robot_control/sample_trajectory.json " << std::endl;
        return -1;
    }

    std::string json_file_path = argv[1];

    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "json_trajectory_executor", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    try {
        // Load joint states from JSON file
        std::vector<JointState> joint_states = load_joint_states_from_json(json_file_path);

        if (joint_states.empty()) {
            RCLCPP_ERROR(node->get_logger(), "No joint states found in JSON file");
            rclcpp::shutdown();
            return -1;
        }

        // Create the MoveIt MoveGroup Interface for the robot arm
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_robot_arm = std::make_shared<MoveGroupInterface>(node, "robot_arm");
        move_group_robot_arm->setPlannerId("RRTConnectConfigDefault");
        move_group_robot_arm->setPlanningTime(100.0);
        move_group_robot_arm->setNumPlanningAttempts(10);

        // Create the MoveIt MoveGroup Interface for the hand (gripper)
        auto move_group_hand = std::make_shared<MoveGroupInterface>(node, "hand");
        
        // Add diagnostic information about the gripper
        RCLCPP_INFO(node->get_logger(), "Gripper group info:");
        RCLCPP_INFO(node->get_logger(), "  Planning frame: %s", move_group_hand->getPlanningFrame().c_str());
        RCLCPP_INFO(node->get_logger(), "  End effector link: %s", move_group_hand->getEndEffectorLink().c_str());
        
        std::vector<std::string> joint_names = move_group_hand->getJointNames();
        RCLCPP_INFO(node->get_logger(), "  Joint names (%zu joints):", joint_names.size());
        for (size_t i = 0; i < joint_names.size(); ++i) {
            RCLCPP_INFO(node->get_logger(), "    [%zu]: %s", i, joint_names[i].c_str());
        }

        // Execute the trajectory
        execute_trajectory(*move_group_robot_arm, *move_group_hand, joint_states, "json_trajectory_executor");

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error during execution: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
