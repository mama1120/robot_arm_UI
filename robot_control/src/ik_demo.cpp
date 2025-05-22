#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>
#include <vector>
#include <cmath>
#include <random>

// Constants
constexpr double MIN_RADIUS     = 0.35;
constexpr double MAX_RADIUS     = 0.50;
constexpr double BALL_RADIUS    = 0.03;
constexpr double APPROACH_DELTA = 0.08;

// “90° about Y” quaternion (w, x, y, z)
const std::vector<double> BALL_ORIENTATION = {0.7071068, 0.0, 0.7071068, 0.0};

// Random generator setup
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> radius_dist(MIN_RADIUS, MAX_RADIUS);
std::uniform_real_distribution<> angle_dist(0.0, 2.0 * M_PI);

/**
 * @brief Create a random pose for the ball on the ground plane.
 *        Position: (r cos θ, r sin θ, BALL_RADIUS)
 *        Orientation: fixed “90° about world‐Y” so that IKFast’s pitch→joint4 mapping works.
 */
geometry_msgs::msg::Pose create_random_ball_pose()
{
  geometry_msgs::msg::Pose pose;
  double r     = radius_dist(gen);
  double theta = angle_dist(gen);

  pose.position.x = r * std::cos(theta);
  pose.position.y = r * std::sin(theta);
  pose.position.z = BALL_RADIUS;

  pose.orientation.w = BALL_ORIENTATION[0];
  pose.orientation.x = BALL_ORIENTATION[1];
  pose.orientation.y = BALL_ORIENTATION[2];
  pose.orientation.z = BALL_ORIENTATION[3];
  return pose;
}

/**
 * @brief Add a sphere to the planning scene at the given pose.
 */
void spawn_ball(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                const geometry_msgs::msg::Pose &ball_pose)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";  // must match MoveIt’s planning frame
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

/**
 * @brief Broadcast a static TF from "base_link" → "target_ball_frame" so you can visualize it.
 */
void publish_ball_tf(const geometry_msgs::msg::Pose &ball_pose, rclcpp::Node::SharedPtr node)
{
  static tf2_ros::StaticTransformBroadcaster tf_broadcaster(node);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "base_link";
  t.child_frame_id  = "target_ball_frame";
  t.transform.translation.x = ball_pose.position.x;
  t.transform.translation.y = ball_pose.position.y;
  t.transform.translation.z = ball_pose.position.z;
  t.transform.rotation      = ball_pose.orientation;

  tf_broadcaster.sendTransform(t);
}

/**
 * @brief Move the arm “down” onto the ball in two steps:
 *        1) Approach from slightly above (z + APPROACH_DELTA).  
 *        2) Move straight down onto the ball.
 *
 * @note For TranslationYAxisAngle4D, MoveIt expects a Pose whose orientation is a “pitch” about world Y.
 */
void move_to_ball(moveit::planning_interface::MoveGroupInterface &move_group,
                  const geometry_msgs::msg::Pose &ball_pose,
                  const std::string &logger_name)
{
  // 1) Approach pose: raise Z by APPROACH_DELTA
  geometry_msgs::msg::Pose approach = ball_pose;
  approach.position.z += APPROACH_DELTA;

  // Make sure we plan from the current state
  move_group.setStartStateToCurrentState();
  move_group.setPoseReferenceFrame("base_link");
  move_group.setPoseTarget(approach);

  RCLCPP_INFO(rclcpp::get_logger(logger_name), "Planning approach motion to (%.3f, %.3f, %.3f)",
              approach.position.x, approach.position.y, approach.position.z);

  bool success_approach = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success_approach) {
    RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Approach motion failed");
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger(logger_name), "Approach successful");

  // 2) Final pose: lower straight down onto the ball
  move_group.setStartStateToCurrentState();
  move_group.setPoseReferenceFrame("base_link");
  move_group.setPoseTarget(ball_pose);

  RCLCPP_INFO(rclcpp::get_logger(logger_name), "Planning final motion to (%.3f, %.3f, %.3f)",
              ball_pose.position.x, ball_pose.position.y, ball_pose.position.z);

  bool success_final = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success_final) {
    RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Final motion failed");
  } else {
    RCLCPP_INFO(rclcpp::get_logger(logger_name), "Final motion successful");
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "ball_picker",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // 1) Initialize MoveIt interfaces
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface move_group(node, "robot_arm");

  // Use RRTConnect (must match your moveit_planning_execution.yaml or similar)
  move_group.setPlannerId("RRTConnectConfigDefault");
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(20);
  move_group.setPoseReferenceFrame("base_link");

  // 2) Generate a random ball pose
  geometry_msgs::msg::Pose ball_pose = create_random_ball_pose();
  RCLCPP_INFO(node->get_logger(), "Spawned ball at: x=%.3f y=%.3f z=%.3f",
              ball_pose.position.x, ball_pose.position.y, ball_pose.position.z);
  RCLCPP_INFO(node->get_logger(), "Ball orientation (wxyz)=%.3f, %.3f, %.3f, %.3f",
              ball_pose.orientation.w, ball_pose.orientation.x,
              ball_pose.orientation.y, ball_pose.orientation.z);

  // 3) Add the ball to the planning scene
  spawn_ball(planning_scene_interface, ball_pose);
  RCLCPP_INFO(node->get_logger(), "Collision object added for ball");

  // Give the planning scene a moment to register the new object
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // 4) Publish a static TF for the ball (for RViz visualization)
  publish_ball_tf(ball_pose, node);
  RCLCPP_INFO(node->get_logger(), "Published static TF for ball as 'target_ball_frame'");

  // 5) Move the arm onto the ball
  move_to_ball(move_group, ball_pose, "ball_picker");

  // 6) Return to “home” named state (make sure "home" is defined in your SRDF for group “robot_arm”)
  move_group.setStartStateToCurrentState();
  move_group.setNamedTarget("home");
  bool success_home = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
  if (success_home) {
    RCLCPP_INFO(node->get_logger(), "Returned to home position");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to return to home");
  }

  rclcpp::shutdown();
  return 0;
}
