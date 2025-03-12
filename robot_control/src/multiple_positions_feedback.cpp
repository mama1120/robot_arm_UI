#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <vector>
#include <cmath>

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double TOLERANCE = 1.0 * DEG2RAD; // 1 degree tolerance in radians

std::vector<double> current_joint_positions(8, 0.0); // To store current joint positions
bool is_initial_movement = true; // Flag to track initial movement

bool within_tolerance(const std::vector<double>& current, const std::vector<double>& target, double tolerance) {
    for (size_t i = 0; i < 4; ++i) { // Only compare first four values (robot arm joints)
        if (std::abs(current[i] - target[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

void log_position_difference(const std::vector<double>& current, const std::vector<double>& target, rclcpp::Logger logger) {
    // Log the differences between current joint positions and target
    std::ostringstream oss;
    oss << "Differences between current and target joint positions: [";
    for (size_t i = 0; i < 4; ++i) {
        oss << (current[i] - target[i]);
        if (i < 3) oss << ", ";
    }
    oss << "]";
    RCLCPP_INFO(logger, oss.str().c_str());
}

void move_to_position(moveit::planning_interface::MoveGroupInterface& move_group_interface, 
                      const std::vector<double>& position, 
                      const std::string& logger_name) {
    move_group_interface.setJointValueTarget(position);
    bool success = static_cast<bool>(move_group_interface.move());
    if (success) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Movement to target position successful");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Movement to target position failed");
    }
}

void initial_movement(moveit::planning_interface::MoveGroupInterface& move_group_interface,
                      rclcpp::Node::SharedPtr node) {
    // Example home position or other known start position
    std::vector<double> initial_position = {0.0, 0.0, 0.0, 0.0}; 
    RCLCPP_INFO(node->get_logger(), "Performing initial movement to a known position...");
    move_to_position(move_group_interface, initial_position, "pose_setter");
}

void current_joint_positions_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() >= 8) {
        current_joint_positions.assign(msg->position.begin(), msg->position.begin() + 8);
        RCLCPP_DEBUG(rclcpp::get_logger("pose_setter"), "Received current joint positions.");
    }
}

void planned_trajectory_callback(const sensor_msgs::msg::JointState::SharedPtr msg,
                                 rclcpp::Node::SharedPtr node,
                                 moveit::planning_interface::MoveGroupInterface& move_group_interface) {
    if (msg->position.size() >= 8) {
        std::vector<double> last_trajectory_point(msg->position.end() - 8, msg->position.end());

        if (is_initial_movement) {
            // Skip the check for the first movement
            is_initial_movement = false; // Ensure this is not repeated
        } else {
            // Log the difference between the current and target positions
            log_position_difference(current_joint_positions, last_trajectory_point, node->get_logger());

            // Check if the robot's current joint positions are within tolerance of the target
            if (within_tolerance(current_joint_positions, last_trajectory_point, TOLERANCE)) {
                RCLCPP_INFO(node->get_logger(), "Current position within tolerance, moving to next trajectory point...");
                move_to_position(move_group_interface, {last_trajectory_point.begin(), last_trajectory_point.begin() + 4}, "pose_setter");
            } else {
                RCLCPP_INFO(node->get_logger(), "Waiting until the robot arm is within tolerance of the target position...");
            }
        }
    }
}

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "pose_setter", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = std::make_shared<MoveGroupInterface>(node, "robot_arm");

    // Perform the initial movement before starting to process callbacks
    initial_movement(*move_group_interface, node);

    // Subscribe to /current_joint_positions
    auto current_joint_positions_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/current_joint_positions", 10, current_joint_positions_callback);

    // Subscribe to /planned_trajectory
    auto planned_trajectory_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/planned_trajectory", 10, 
        [node, move_group_interface](const sensor_msgs::msg::JointState::SharedPtr msg) {
            planned_trajectory_callback(msg, node, *move_group_interface);
        });

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
