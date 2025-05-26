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
#include <sensor_msgs/msg/joint_state.hpp>

// Constants with corrected orientation
constexpr double MIN_RADIUS = 0.38;
constexpr double MAX_RADIUS = 0.50;
constexpr double BALL_RADIUS = 0.02;
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
    
    // Clear any previous pose targets
    move_group.clearPoseTargets();
    
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

    
    // Initialize MoveIt interfaces
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group(node, "robot_arm");
    move_group.setPlannerId("RRTConnectConfigDefault");
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(20);

    auto spin_thread = std::thread([&executor]() {
        while (rclcpp::ok()) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    
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

    // Move to ball position
    move_to_ball(move_group, ball_pose, "ball_picker");
    
    // Calculate and display the transform difference
    calculate_transform_difference(node, ball_pose);
    
    // Wait to ensure we can see the output and receive joint states
    RCLCPP_INFO(node->get_logger(), "Waiting for joint state updates...");
    rclcpp::sleep_for(std::chrono::seconds(1));
        
// Print joint states from the ROS topic
if (latest_joint_state != nullptr) {
    RCLCPP_INFO(node->get_logger(), "Current joint states:");
    
    for (size_t i = 0; i < latest_joint_state->name.size(); ++i) {
        const std::string &joint_name = latest_joint_state->name[i];
        double position_rad = latest_joint_state->position[i];
        
        double value = 0.0;
        std::string value_str;
        
        if (joint_name == "joint_plate") {
            // Convert joint_plate position to percentage
            double mm = position_rad * 1000.0;
            int percent = static_cast<int>(std::round(mm / 10.0 * 100.0));
            percent = std::clamp(percent, 0, 100);
            value = percent;
            value_str = std::to_string(percent) + "%";
        } else {
            // Convert radians to degrees for other joints
            value = position_rad * 180.0 / M_PI;  // deg
            std::stringstream ss;
            ss << std::fixed << std::setprecision(1) << value << "°";
            value_str = ss.str();
        }
        
        RCLCPP_INFO(node->get_logger(), "  %s: %s", joint_name.c_str(), value_str.c_str());
    }
} else {
    RCLCPP_WARN(node->get_logger(), "No joint state messages received yet");
}

    // Adjust joint4 orientation by publishing to target_joint_states topic
    adjust_joint4_via_topic(joint_pub, latest_joint_state, node, ball_pose);
    
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
    
    rclcpp::shutdown();
    return 0;
}