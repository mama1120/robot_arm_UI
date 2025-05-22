#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <vector>
#include <cmath>
#include <random>
// Constants with corrected orientation
constexpr double MIN_RADIUS = 0.35;
constexpr double MAX_RADIUS = 0.50;
constexpr double BALL_RADIUS = 0.03;
constexpr double APPROACH_HEIGHT = 0.08;
// Corrected 90° rotation about Y-axis (positive direction)
const std::vector<double> BALL_ORIENTATION = {0.707, 0.0, 0.707, 0.0};  // w, x, y, z

// Random number generator setup
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> radius_dist(MIN_RADIUS, MAX_RADIUS);
std::uniform_real_distribution<> angle_dist(0.0, 2.0 * M_PI);

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
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "target_ball";

    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = sphere.SPHERE;
    sphere.dimensions.resize(1);
    sphere.dimensions[0] = BALL_RADIUS;

    collision_object.primitives.push_back(sphere);
    collision_object.primitive_poses.push_back(ball_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface.applyCollisionObject(collision_object);
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
void move_to_ball(moveit::planning_interface::MoveGroupInterface& move_group, 
                 const geometry_msgs::msg::Pose& ball_pose,
                 const std::string& logger_name) {
    // Approach position (currently not used)
    geometry_msgs::msg::Pose approach_pose = ball_pose;
    approach_pose.position.x -= APPROACH_HEIGHT;
    
    // Move to approach position
    move_group.setPoseTarget(approach_pose);
    if(move_group.move()) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Approach movement successful");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Approach movement failed");
        return;
    }
    
    // Move to final position
    move_group.setPoseTarget(ball_pose);
    if(move_group.move()) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), "Final movement successful");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Final movement failed");
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "ball_picker", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Initialize MoveIt interfaces
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group(node, "robot_arm");
    move_group.setPlannerId("RRTConnectConfigDefault");
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(20);

    // Generate random ball position
    geometry_msgs::msg::Pose ball_pose = create_random_ball_pose();
    
    // Spawn ball in the scene
    spawn_ball(planning_scene_interface, ball_pose);
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

    // Return to home position
    move_group.setNamedTarget("home");
    if(move_group.move()) {
        RCLCPP_INFO(node->get_logger(), "Returned to home position");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Home position movement failed");
    }

    rclcpp::shutdown();
    return 0;
}