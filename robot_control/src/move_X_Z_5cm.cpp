#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Function for moving to any other position
void move_to_position(moveit::planning_interface::MoveGroupInterface& move_group, 
                      const geometry_msgs::msg::Pose& target_pose,
                      const std::string& logger_name) {
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

    // Create a TF buffer and listener
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Define the relative movement (e.g., 5 cm in Z direction)
    double relative_movement_z = -0.05; // 5 cm in Z direction

    // Get the transform from link4_1 to base_link
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer.lookupTransform("base_link", "link4_1", tf2::TimePointZero, tf2::durationFromSec(1.0));
        RCLCPP_INFO(node->get_logger(), "Transform from link4_1 to base_link: translation=(%.3f, %.3f, %.3f), rotation=(%.3f, %.3f, %.3f, %.3f)",
                    transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z,
                    transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(node->get_logger(), "TF lookup failed: %s", ex.what());
        rclcpp::shutdown();
        return 1;
    }

    // Create a pose in the link4_1 frame with the relative movement
    geometry_msgs::msg::PoseStamped pose_link4_1;
    pose_link4_1.header.frame_id = "link4_1";
    pose_link4_1.pose.position.x = 0.0;
    pose_link4_1.pose.position.y = 0.0;
    pose_link4_1.pose.position.z = relative_movement_z; // Add 5 cm in Z direction
    pose_link4_1.pose.orientation.w = 1.0; // No rotation

    RCLCPP_INFO(node->get_logger(), "Pose in link4_1 (after adding relative movement): x=%.3f, y=%.3f, z=%.3f",
                pose_link4_1.pose.position.x, pose_link4_1.pose.position.y, pose_link4_1.pose.position.z);

    // Transform the pose back to the base_link frame
    geometry_msgs::msg::PoseStamped pose_base_link;
    try {
        pose_base_link = tf_buffer.transform(pose_link4_1, "base_link", tf2::durationFromSec(1.0));
        RCLCPP_INFO(node->get_logger(), "Pose in base_link (after transformation back): x=%.3f, y=%.3f, z=%.3f",
                    pose_base_link.pose.position.x, pose_base_link.pose.position.y, pose_base_link.pose.position.z);
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(node->get_logger(), "TF transformation failed: %s", ex.what());
        rclcpp::shutdown();
        return 1;
    }

    // Move the robot arm to the new pose
    RCLCPP_INFO(node->get_logger(), "Moving to new pose: x=%.3f, y=%.3f, z=%.3f",
                pose_base_link.pose.position.x, pose_base_link.pose.position.y, pose_base_link.pose.position.z);
    move_to_position(*move_group_robot_arm, pose_base_link.pose, "pose_setter");

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}