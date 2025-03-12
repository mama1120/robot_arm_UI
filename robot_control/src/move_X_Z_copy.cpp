#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>

// Function for moving to any other position
void move_to_position(moveit::planning_interface::MoveGroupInterface& move_group, 
                      const std::vector<double>& position,
                      const double& approach, 
                      const std::string& logger_name) {
    // Define the target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = position[0];
    target_pose.position.y = position[1];
    target_pose.position.z = position[2] + approach;
    target_pose.orientation.w = position[3];
    target_pose.orientation.x = position[4];
    target_pose.orientation.y = position[5];
    target_pose.orientation.z = position[6];

    // Set planner and planning parameters
    move_group.setPlannerId("RRTConnectConfigDefault");
    move_group.setNumPlanningAttempts(5);

    // Set the target pose
    move_group.setPoseTarget(target_pose);

    // Plan and execute the movement
    bool success = static_cast<bool>(move_group.move());
    if (success) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Movement successful");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Movement failed");
    }
}

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "pose_setter", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create the MoveIt MoveGroup Interface for the robot arm
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_robot_arm = std::make_shared<MoveGroupInterface>(node, "robot_arm");
    move_group_robot_arm->setPlannerId("RRTConnectConfigDefault");
    move_group_robot_arm->setPlanningTime(100.0);
    move_group_robot_arm->setNumPlanningAttempts(10);

    // Define a target position (example values)
    std::vector<double> target_position = {0.331, 0.340, 0.368, 0.000, -0.000, 0.379, 0.925};
    double approach_value = 0.1; // 10 cm approach
    geometry_msgs::msg::PoseStamped ee_pose = move_group_robot_arm->getCurrentPose();
    RCLCPP_INFO(node->get_logger(), "End Effector Pose: x=%.3f, y=%.3f, z=%.3f", 
                ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z);
    
    // Move the robot arm to the target position
    RCLCPP_INFO(node->get_logger(), "Moving to target position...");
    move_to_position(*move_group_robot_arm, target_position, approach_value, "pose_setter");

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}