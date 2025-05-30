#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <memory>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>
#include <sstream>
#include <thread>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/constraints.hpp>

// Constants with corrected orientation
constexpr double MIN_RADIUS = 0.38;
constexpr double MAX_RADIUS = 0.50;
constexpr double BALL_RADIUS = 0.015;  // 15mm ball radius
constexpr double APPROACH_HEIGHT = 0.10;
// Corrected 90° rotation about Y-axis (positive direction)
const std::vector<double> BALL_ORIENTATION = {0.707, 0.0, 0.707, 0.0};  // w, x, y, z

// Random number generator setup
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> radius_dist(MIN_RADIUS, MAX_RADIUS);
std::uniform_real_distribution<> angle_dist(0.0, 2.0 * M_PI);

// Helper function to convert quaternion to euler angles (roll, pitch, yaw)
std::vector<double> quaternion_to_euler(const geometry_msgs::msg::Quaternion& q) {
    // Convert quaternion to Euler angles (roll, pitch, yaw)
    // Roll (x-axis rotation)
    double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 
                           1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    
    // Pitch (y-axis rotation)
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    double pitch;
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);
    
    // Yaw (z-axis rotation)
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    
    // Return as radians
    return {roll, pitch, yaw};
}

// Helper function to convert quaternion to euler angles with Y-Z-X sequence (pitch, yaw, roll)
std::vector<double> quaternion_to_euler_yzx(const geometry_msgs::msg::Quaternion& q) {
    // Convert quaternion to Euler angles using YZX rotation sequence
    // Pitch (y-axis rotation) - first rotation
    double pitch = std::atan2(2.0 * (q.w * q.y + q.z * q.x), 
                            1.0 - 2.0 * (q.y * q.y + q.x * q.x));
    
    // Yaw (z-axis rotation) - second rotation
    double sinYaw = 2.0 * (q.w * q.z - q.x * q.y);
    double yaw;
    if (std::abs(sinYaw) >= 1)
        yaw = std::copysign(M_PI / 2, sinYaw); // use 90 degrees if out of range
    else
        yaw = std::asin(sinYaw);
    
    // Roll (x-axis rotation) - third rotation
    double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 
                           1.0 - 2.0 * (q.x * q.x + q.z * q.z));
    
    // Return as radians - pitch (Y), yaw (Z), roll (X)
    return {pitch, yaw, roll};
}

geometry_msgs::msg::Pose create_random_ball_pose() {
    geometry_msgs::msg::Pose pose;
    double radius = radius_dist(gen);
    double angle = angle_dist(gen);
    
    // Convert polar to Cartesian coordinates
    pose.position.x = radius * cos(angle);
    pose.position.y = radius * sin(angle);
    pose.position.z = BALL_RADIUS;  // Ground level + ball radius
    
    // Set fixed orientation
    pose.orientation.w = BALL_ORIENTATION[0];
    pose.orientation.x = BALL_ORIENTATION[1];
    pose.orientation.y = BALL_ORIENTATION[2];
    pose.orientation.z = BALL_ORIENTATION[3];
    
    return pose;
}

void spawn_ball(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, 
                const geometry_msgs::msg::Pose& ball_pose) {
    // First create the collision object
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "target_ball";

    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = sphere.SPHERE;
    sphere.dimensions.resize(1);
    sphere.dimensions[0] = BALL_RADIUS;

    collision_object.primitives.push_back(sphere);
    collision_object.primitive_poses.push_back(ball_pose);
    
    // If we want to visualize but don't want collision checking
    collision_object.id = "visual_marker_target_ball";

    
    collision_object.operation = collision_object.ADD;
    
    // Apply the collision object to the planning scene
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects{collision_object};
    planning_scene_interface.applyCollisionObjects(collision_objects);

}

void move_to_ball(moveit::planning_interface::MoveGroupInterface& move_group, 
                 const geometry_msgs::msg::Pose& ball_pose,
                 const std::string& logger_name) {
    // Set position-only tolerances
    move_group.setGoalPositionTolerance(0.01);  // 1cm position tolerance
    
    // Calculate approach position
    geometry_msgs::msg::Point approach_position;
    approach_position.x = ball_pose.position.x ;
    approach_position.y = ball_pose.position.y;
    approach_position.z = ball_pose.position.z + APPROACH_HEIGHT;
    
    // Clear any previous pose targets and constraints
    move_group.clearPoseTargets();
    move_group.clearPathConstraints();
    
    // Set approach position target (position only)
    RCLCPP_INFO(rclcpp::get_logger(logger_name), 
                "Moving to approach position (%.2f, %.2f, %.2f)",
                approach_position.x, approach_position.y, approach_position.z);
    
    move_group.setPositionTarget(
        approach_position.x, 
        approach_position.y, 
        approach_position.z, 
        move_group.getEndEffectorLink()
    );
    
    // Execute approach movement
    auto approach_result = move_group.move();
    
    if(approach_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Approach movement successful");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Approach movement failed");
        return;
    }
    
    // Clear targets again
    move_group.clearPoseTargets();
}

void move_to_ball_keep_joint4_fixed(moveit::planning_interface::MoveGroupInterface& move_group, 
                                    const geometry_msgs::msg::Pose& ball_pose,
                                    const sensor_msgs::msg::JointState::SharedPtr& latest_joint_state,
                                    const std::string& logger_name) {
    // Set position-only tolerances
    move_group.setGoalPositionTolerance(0.01);  // 1cm position tolerance
    
    // Calculate approach position
    geometry_msgs::msg::Point approach_position;
    approach_position.x = ball_pose.position.x;
    approach_position.y = ball_pose.position.y;
    approach_position.z = ball_pose.position.z + APPROACH_HEIGHT;
    
    // Clear any previous pose targets and constraints
    move_group.clearPoseTargets();
    move_group.clearPathConstraints();
    
    // Get current joint4 value to fix it during planning
    double current_joint4_value = 0.0;
    if (latest_joint_state != nullptr) {
        for (size_t i = 0; i < latest_joint_state->name.size(); ++i) {
            if (latest_joint_state->name[i] == "joint4") {
                current_joint4_value = latest_joint_state->position[i];
                break;
            }
        }
    }
    
    // Create joint constraint to keep joint4 fixed
    moveit_msgs::msg::Constraints joint_constraints;
    moveit_msgs::msg::JointConstraint joint4_constraint;
    joint4_constraint.joint_name = "joint4";
    joint4_constraint.position = current_joint4_value;
    joint4_constraint.tolerance_above = 0.01;  // Small tolerance (about 0.6 degrees)
    joint4_constraint.tolerance_below = 0.01;
    joint4_constraint.weight = 1.0;  // High priority
    
    joint_constraints.joint_constraints.push_back(joint4_constraint);
    
    // Apply the constraints
    move_group.setPathConstraints(joint_constraints);
    
    RCLCPP_INFO(rclcpp::get_logger(logger_name), 
                "Moving to approach position (%.2f, %.2f, %.2f) while keeping joint4 fixed at %.3f rad (%.1f deg)",
                approach_position.x, approach_position.y, approach_position.z,
                current_joint4_value, current_joint4_value * 180.0 / M_PI);
    
    // Set approach position target (position only)
    move_group.setPositionTarget(
        approach_position.x, 
        approach_position.y, 
        approach_position.z, 
        move_group.getEndEffectorLink()
    );
    
    // Execute approach movement
    auto approach_result = move_group.move();
    
    if(approach_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Approach movement successful with joint4 constraint");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Approach movement failed with joint4 constraint");
    }
    
    // Clear targets and constraints
    move_group.clearPoseTargets();
    move_group.clearPathConstraints();
}

void move_to_ball_using_joint_targets(moveit::planning_interface::MoveGroupInterface& move_group, 
                                      const geometry_msgs::msg::Pose& ball_pose,
                                      const sensor_msgs::msg::JointState::SharedPtr& latest_joint_state,
                                      const std::string& logger_name) {
    if (latest_joint_state == nullptr || latest_joint_state->name.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "No joint state information available");
        return;
    }
    
    // Calculate approach position
    geometry_msgs::msg::Point approach_position;
    approach_position.x = ball_pose.position.x;
    approach_position.y = ball_pose.position.y;
    approach_position.z = ball_pose.position.z + APPROACH_HEIGHT;
    
    // Clear any previous targets and constraints
    move_group.clearPoseTargets();
    move_group.clearPathConstraints();
    
    // Get current joint values
    std::map<std::string, double> current_joints;
    for (size_t i = 0; i < latest_joint_state->name.size(); ++i) {
        current_joints[latest_joint_state->name[i]] = latest_joint_state->position[i];
    }
    
    // Set position target first to get the robot close to the desired position
    RCLCPP_INFO(rclcpp::get_logger(logger_name), 
                "First step: Moving to approach position (%.2f, %.2f, %.2f) keeping joint4 at %.3f rad (%.1f deg)",
                approach_position.x, approach_position.y, approach_position.z,
                current_joints["joint4"], current_joints["joint4"] * 180.0 / M_PI);
    
    // Method 2A: Use setJointValueTarget with partial joint specification
    // This allows you to specify values for some joints while letting the planner 
    // figure out the others to reach the position target
    
    // First, set the position target
    move_group.setPositionTarget(
        approach_position.x, 
        approach_position.y, 
        approach_position.z, 
        move_group.getEndEffectorLink()
    );
    
    // Then constrain joint4 to its current value
    move_group.setJointValueTarget("joint4", current_joints["joint4"]);
    
    // Execute the movement
    auto approach_result = move_group.move();
    
    if(approach_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Approach movement successful using joint value target for joint4");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Approach movement failed using joint value target for joint4");
    }
    
    // Clear targets
    move_group.clearPoseTargets();
}

void calculate_transform_difference(const rclcpp::Node::SharedPtr& node, 
                                   const geometry_msgs::msg::Pose& ball_pose) {
    // Create a TF buffer and listener
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);
    
    // Wait a bit for transforms to be available
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    try {
        // Get the transform from base_link to link4_1 (end effector)
        geometry_msgs::msg::TransformStamped link4_1_transform = 
            tf_buffer.lookupTransform("base_link", "link4_1", tf2::TimePointZero);
        
        // Create a transform for the ball (which is in base_link frame already)
        geometry_msgs::msg::TransformStamped ball_transform;
        ball_transform.header.frame_id = "base_link";
        ball_transform.child_frame_id = "ball";
        ball_transform.transform.translation.x = ball_pose.position.x;
        ball_transform.transform.translation.y = ball_pose.position.y;
        ball_transform.transform.translation.z = ball_pose.position.z;
        ball_transform.transform.rotation = ball_pose.orientation;
        
        // Convert to Eigen for easier calculations
        Eigen::Isometry3d link4_1_eigen = tf2::transformToEigen(link4_1_transform);
        Eigen::Isometry3d ball_eigen = tf2::transformToEigen(ball_transform);
        
        // Calculate difference in base_link frame
        Eigen::Isometry3d diff_base = ball_eigen.inverse() * link4_1_eigen;
        
        // Convert back to ROS message
        geometry_msgs::msg::TransformStamped diff_base_msg = tf2::eigenToTransform(diff_base);
        
        // Calculate difference in link4_1 frame
        Eigen::Isometry3d diff_link4_1 = link4_1_eigen.inverse() * ball_eigen;
        
        // Convert back to ROS message
        geometry_msgs::msg::TransformStamped diff_link4_1_msg = tf2::eigenToTransform(diff_link4_1);
        
        // Convert quaternions to Euler angles using YZX sequence
        auto euler_base_yzx = quaternion_to_euler_yzx(diff_base_msg.transform.rotation);
        auto euler_link4_1_yzx = quaternion_to_euler_yzx(diff_link4_1_msg.transform.rotation);
        
        
        // Print results in link4_1 frame
        RCLCPP_INFO(node->get_logger(), "Difference in link4_1 frame:");
        RCLCPP_INFO(node->get_logger(), "Position difference (x,y,z): (%.3f, %.3f, %.3f)",
                   diff_link4_1_msg.transform.translation.x,
                   diff_link4_1_msg.transform.translation.y,
                   diff_link4_1_msg.transform.translation.z);
        RCLCPP_INFO(node->get_logger(), "Orientation difference (pitch,yaw,roll) YZX [deg]: (%.1f, %.1f, %.1f)",
                   euler_link4_1_yzx[0] * 180.0 / M_PI, 
                   euler_link4_1_yzx[1] * 180.0 / M_PI, 
                   euler_link4_1_yzx[2] * 180.0 / M_PI);
        
        // Highlight the y-axis rotation in link4_1 frame (what joint4 can adjust)
        RCLCPP_INFO(node->get_logger(), "Required joint4 adjustment (y-axis rotation in link4_1 frame):");
        RCLCPP_INFO(node->get_logger(), "%.3f radians (%.1f degrees)",
                   euler_link4_1_yzx[0], euler_link4_1_yzx[0] * 180.0 / M_PI);
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(node->get_logger(), "Transform error: %s", ex.what());
    }
}

void publish_ball_tf(const geometry_msgs::msg::Pose& ball_pose, rclcpp::Node::SharedPtr node) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster(node);
    
    geometry_msgs::msg::TransformStamped transform_stamped;
    
    transform_stamped.header.stamp = node->now();
    transform_stamped.header.frame_id = "base_link";
    transform_stamped.child_frame_id = "target_ball_frame";
    
    transform_stamped.transform.translation.x = ball_pose.position.x;
    transform_stamped.transform.translation.y = ball_pose.position.y;
    transform_stamped.transform.translation.z = ball_pose.position.z;
    
    transform_stamped.transform.rotation = ball_pose.orientation;
    
    static_broadcaster.sendTransform(transform_stamped);
}

// Fix for adjust_joint4_via_topic function
void adjust_joint4_via_topic(const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& joint_pub,
                             const sensor_msgs::msg::JointState::SharedPtr& latest_joint_state,
                             const rclcpp::Node::SharedPtr& node,
                             const geometry_msgs::msg::Pose& ball_pose) {
    RCLCPP_INFO(node->get_logger(), "Adjusting joint4 by publishing to target_joint_states");
    
    if (latest_joint_state == nullptr || latest_joint_state->name.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No joint state information available");
        return;
    }
    
    // Create a TF buffer and listener
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);
    
    try {
        // Wait a bit for transforms to be available
        rclcpp::sleep_for(std::chrono::seconds(1));
        
        // Get the transform from base_link to link4_1 (end effector)
        geometry_msgs::msg::TransformStamped link4_1_transform = 
            tf_buffer.lookupTransform("base_link", "link4_1", tf2::TimePointZero);
        
        // Create a transform for the ball (which is in base_link frame already)
        geometry_msgs::msg::TransformStamped ball_transform;
        ball_transform.header.frame_id = "base_link";
        ball_transform.child_frame_id = "ball";
        ball_transform.transform.translation.x = ball_pose.position.x;
        ball_transform.transform.translation.y = ball_pose.position.y;
        ball_transform.transform.translation.z = ball_pose.position.z;
        ball_transform.transform.rotation = ball_pose.orientation;
        
        // Convert to Eigen for easier calculations
        Eigen::Isometry3d link4_1_eigen = tf2::transformToEigen(link4_1_transform);
        Eigen::Isometry3d ball_eigen = tf2::transformToEigen(ball_transform);
        
        // Calculate difference in link4_1 frame
        Eigen::Isometry3d diff_link4_1 = link4_1_eigen.inverse() * ball_eigen;
        
        // Convert back to ROS message
        geometry_msgs::msg::TransformStamped diff_link4_1_msg = tf2::eigenToTransform(diff_link4_1);
        
        // Get the Y-axis rotation (pitch) in YZX sequence
        auto euler_link4_1_yzx = quaternion_to_euler_yzx(diff_link4_1_msg.transform.rotation);
        double y_rotation_adjustment = euler_link4_1_yzx[0]; // First angle is Y in YZX sequence
        
        RCLCPP_INFO(node->get_logger(), "Required joint4 adjustment: %.3f radians (%.1f degrees)",
                   y_rotation_adjustment, y_rotation_adjustment * 180.0 / M_PI);
        
        // Find joint4 index in the joint_states message
        int joint4_index = -1;
        for (size_t i = 0; i < latest_joint_state->name.size(); ++i) {
            if (latest_joint_state->name[i] == "joint4") {
                joint4_index = static_cast<int>(i);
                break;
            }
        }
        
        if (joint4_index == -1) {
            RCLCPP_ERROR(node->get_logger(), "joint4 not found in joint states");
            return;
        }
        
        // Current joint4 value
        double current_joint4_value = latest_joint_state->position[joint4_index];
        RCLCPP_INFO(node->get_logger(), "Current joint4 value: %.3f radians (%.1f degrees)",
                   current_joint4_value, current_joint4_value * 180.0 / M_PI);
        
        // Calculate new joint4 value by adding the adjustment
        double new_joint4_value = current_joint4_value + y_rotation_adjustment;
        
        // Define joint4 limits in radians
        const double JOINT4_MIN_RAD = -120.0 * M_PI / 180.0;  // -120 degrees
        const double JOINT4_MAX_RAD = 75.0 * M_PI / 180.0;    // 75 degrees
        
        // First check if direct adjustment is within limits
        if (new_joint4_value > JOINT4_MAX_RAD || new_joint4_value < JOINT4_MIN_RAD) {
            // If adding the adjustment puts us outside limits, try the opposite direction
            double alternative_adjustment = y_rotation_adjustment;
            // Calculate the complementary angle (turn the other way)
            if (y_rotation_adjustment > 0) {
                alternative_adjustment = y_rotation_adjustment - M_PI;  // Subtract 180°
            } else {
                alternative_adjustment = y_rotation_adjustment + M_PI;  // Add 180°
            }
            
            RCLCPP_INFO(node->get_logger(), "Trying alternative rotation direction: %.3f radians (%.1f degrees)",
                        alternative_adjustment, alternative_adjustment * 180.0 / M_PI);
            
            new_joint4_value = current_joint4_value + alternative_adjustment;
        
        }
        
        // Final limit check and clamping
        if (new_joint4_value > JOINT4_MAX_RAD) {
            RCLCPP_WARN(node->get_logger(), "Clamping joint4 value to maximum (%.1f degrees)", 
                       JOINT4_MAX_RAD * 180.0 / M_PI);
            new_joint4_value = JOINT4_MAX_RAD;
        } else if (new_joint4_value < JOINT4_MIN_RAD) {
            RCLCPP_WARN(node->get_logger(), "Clamping joint4 value to minimum (%.1f degrees)", 
                       JOINT4_MIN_RAD * 180.0 / M_PI);
            new_joint4_value = JOINT4_MIN_RAD;
        }
        
        RCLCPP_INFO(node->get_logger(), "New joint4 value: %.3f radians (%.1f degrees)",
                   new_joint4_value, new_joint4_value * 180.0 / M_PI);
                   
        // Create joint state message to publish
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = node->now();
        msg.header.frame_id = "joint4_adjustment_command";
        
        // Order must match what the controller expects
        msg.name = {"joint2", "joint3", "joint1", "joint4", "joint_plate"};
        
        // Initialize positions with current values
        msg.position = {0.0, 0.0, 0.0, 0.0, 0.0};
        
        // Update values from the latest joint state
        for (size_t i = 0; i < latest_joint_state->name.size(); ++i) {
            const std::string& name = latest_joint_state->name[i];
            double position = latest_joint_state->position[i];
            
            // Find index in our message
            for (size_t j = 0; j < msg.name.size(); ++j) {
                if (msg.name[j] == name) {
                    // If it's joint4, use our new value
                    if (name == "joint4") {
                        msg.position[j] = new_joint4_value;
                    }
                    // For other joints, keep the current position
                    else {
                        msg.position[j] = position;
                    }
                    break;
                }
            }
        }
        // Set the gripper to fully open (1.0)
        // Find the index of joint_plate in our message
        for (size_t j = 0; j < msg.name.size(); ++j) {
            if (msg.name[j] == "joint_plate") {
                // 1.0 is fully open in the GripperCommand (as shown in the listener code)
                msg.position[j] = 1.0;  // Fully open gripper
                RCLCPP_INFO(node->get_logger(), "Opening gripper (setting joint_plate to 1.0)");
                break;
            }
        }
        
        // Publish the message
        joint_pub->publish(msg);
        RCLCPP_INFO(node->get_logger(), "Published new joint state with adjusted joint4 value");
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(node->get_logger(), "Transform error: %s", ex.what());
    }
}
 
// Simple function to open the gripper
void open_gripper(const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& joint_pub,
                  const sensor_msgs::msg::JointState::SharedPtr& latest_joint_state,
                  const rclcpp::Node::SharedPtr& node) {
    RCLCPP_INFO(node->get_logger(), "Opening gripper...");
    
    if (latest_joint_state == nullptr || latest_joint_state->name.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No joint state information available");
        return;
    }
    
    // Create joint state message to publish
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node->now();
    msg.header.frame_id = "gripper_open_command";
    
    // Order must match what the controller expects
    msg.name = {"joint2", "joint3", "joint1", "joint4", "joint_plate"};
    
    // Initialize positions with current values
    msg.position = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    // Update values from the latest joint state
    for (size_t i = 0; i < latest_joint_state->name.size(); ++i) {
        const std::string& name = latest_joint_state->name[i];
        double position = latest_joint_state->position[i];
        
        // Find index in our message
        for (size_t j = 0; j < msg.name.size(); ++j) {
            if (msg.name[j] == name) {
                msg.position[j] = position;
                break;
            }
        }
    }
    
    // Set the gripper to fully open (1.0)
    for (size_t j = 0; j < msg.name.size(); ++j) {
        if (msg.name[j] == "joint_plate") {
            msg.position[j] = 0.01;  // Fully open gripper
            RCLCPP_INFO(node->get_logger(), "Setting joint_plate to 1.0 (fully open)");
            break;
        }
    }
    
    // Publish the message
    joint_pub->publish(msg);
    RCLCPP_INFO(node->get_logger(), "Published gripper open command");
}

// Simplified function to move to ball position using tcp_link
void move_to_ball_simple(moveit::planning_interface::MoveGroupInterface& move_group, 
                        const geometry_msgs::msg::Pose& ball_pose,
                        const std::string& logger_name) {
    // Set position-only tolerances
    move_group.setGoalPositionTolerance(0.01);  // 1cm position tolerance
    
    // Calculate target position (directly at ball position)
    geometry_msgs::msg::Point target_position;
    target_position.x = ball_pose.position.x;
    target_position.y = ball_pose.position.y;
    target_position.z = ball_pose.position.z;
    
    // Clear any previous pose targets and constraints
    move_group.clearPoseTargets();
    move_group.clearPathConstraints();
    
    RCLCPP_INFO(rclcpp::get_logger(logger_name), 
                "Moving link4_1 to ball position (%.2f, %.2f, %.2f)",
                target_position.x, target_position.y, target_position.z);
    
    // Set target position (position only, let MoveIt handle orientation)
    move_group.setPositionTarget(
        target_position.x, 
        target_position.y, 
        target_position.z, 
        move_group.getEndEffectorLink()
    );
    
    // Execute movement
    auto result = move_group.move();
    
    if(result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Successfully moved to ball position");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Failed to move to ball position");
    }
    
    // Clear targets
    move_group.clearPoseTargets();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "ball_picker", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Joint state publisher and subscriber
    auto joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("target_joint_states", 10);
    
    // Storage for the latest joint state message
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state = nullptr;
    
    // Create joint state subscriber
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(),  // Note: removed leading slash
        [&latest_joint_state](const sensor_msgs::msg::JointState::SharedPtr msg) {
            latest_joint_state = msg;
        });
    
    // Wait a moment to receive some joint state messages
    RCLCPP_INFO(node->get_logger(), "Waiting for joint state messages...");
    
    // Create an executor and add the node to it
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    // Process callbacks for 2 seconds to collect joint state messages
    auto start_time = node->now();
    while (rclcpp::ok() && (node->now() - start_time).seconds() < 2.0) {
        executor.spin_some(std::chrono::milliseconds(100));
    }

    // Start the spin thread after initial setup
    auto spin_thread = std::thread([&executor]() {
        while (rclcpp::ok()) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    
    // Initialize MoveIt interfaces
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group(node, "robot_arm");
    move_group.setPlannerId("RRTConnectConfigDefault");
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(20);
    
    // Set link4_1 as the end effector (tcp_link not recognized by MoveIt)
    move_group.setEndEffectorLink("tcp_link");
    
    // Generate random ball position
    geometry_msgs::msg::Pose ball_pose = create_random_ball_pose();
    
    // Spawn ball in the scene
    spawn_ball(planning_scene_interface, ball_pose);  // false to disable collision checking for visualization
    RCLCPP_INFO(node->get_logger(), "Spawned ball at: (%.2f, %.2f, %.2f)", 
                ball_pose.position.x, ball_pose.position.y, ball_pose.position.z);
    RCLCPP_INFO(node->get_logger(), "Ball orientation: (%.2f, %.2f, %.2f, %.2f)",
                ball_pose.orientation.w, ball_pose.orientation.x, 
                ball_pose.orientation.y, ball_pose.orientation.z);
    RCLCPP_INFO(node->get_logger(), "Ball radius: %.2f", BALL_RADIUS);

    // Publish static TF for visualization
    publish_ball_tf(ball_pose, node);
    RCLCPP_INFO(node->get_logger(), "Published static TF for ball visualization");

    // Simplified sequence:
    // 1. Open the gripper
    // 2. Move to ball position with tcp_link
    
    RCLCPP_INFO(node->get_logger(), "\n=== Step 1: Opening gripper ===");
    open_gripper(joint_pub, latest_joint_state, node);
    
    // Wait for gripper to open
    RCLCPP_INFO(node->get_logger(), "Waiting for gripper to open...");
    rclcpp::sleep_for(std::chrono::seconds(3));
    
    RCLCPP_INFO(node->get_logger(), "\n=== Step 2: Moving link4_1 to ball position ===");
    move_to_ball_simple(move_group, ball_pose, "ball_picker");
    
    // Optional: Return to home position
    /*
    RCLCPP_INFO(node->get_logger(), "Returning to home position");
    move_group.setNamedTarget("home");
    auto home_result = move_group.move();
    
    if (home_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Returned to home position");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Home position movement failed");
    }
    */
    
    RCLCPP_INFO(node->get_logger(), "Task completed successfully!");
    
    // Clean shutdown
    rclcpp::shutdown();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    
    return 0;
}