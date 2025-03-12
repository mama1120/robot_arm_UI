#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "robot_arm_execution",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  
  auto const logger = rclcpp::get_logger("robot_arm_execution");
  
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto robot_arm_move_group = MoveGroupInterface(node, "robot_arm");
  
  using moveit::planning_interface::MoveGroupInterface;
  auto hand_move_group = MoveGroupInterface(node, "hand");
  
  robot_arm_move_group.setPlannerId("RRTConnectConfigDefault");//"LBKPIECEkConfigDefault");
  robot_arm_move_group.setPlanningTime(5.0);
  
  
  robot_arm_move_group.setNamedTarget("home");
  
  moveit::planning_interface::MoveGroupInterface::Plan plan_home_position;
  bool success_home = (robot_arm_move_group.plan(plan_home_position) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success_home) {
    robot_arm_move_group.execute(plan_home_position);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for Moving robot arm to home position");
  }
  
  // Set a target Position (x, y, z)
  /*double target_x = 0.289868026971817;
  double target_y = -0.2498214691877365;
  double target_z = 0.23730303347110748;
  
  move_group_interface.setPositionTarget(target_x, target_y, target_z);*/
  
  // Approach Point
  geometry_msgs::msg::Pose approach_pose;
  approach_pose.position.x = -0.3391999304294586;
  approach_pose.position.y = 0.3847830891609192;
  approach_pose.position.z = 0.28593285977840424; //0.18593285977840424;
  approach_pose.orientation.w = 0.2997700870037079;
  approach_pose.orientation.x = -0.634311318397522;
  approach_pose.orientation.y = 0.29266050457954407;
  approach_pose.orientation.z = 0.6497206091880798;
  
  robot_arm_move_group.setPoseTarget(approach_pose);
  
  auto const [success_approach_point, plan_approach_point] = [&robot_arm_move_group]{
    moveit::planning_interface::MoveGroupInterface::Plan approach_pose;
    auto const ok = static_cast<bool>(robot_arm_move_group.plan(approach_pose));
    return std::make_pair(ok, approach_pose);
  }();
  
  if (success_approach_point) {
    robot_arm_move_group.execute(plan_approach_point);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for Moving robot arm to approach position");
  }
  
  // Picking Position
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = -0.3391999304294586;
  target_pose.position.y = 0.3847830891609192;
  target_pose.position.z = 0.18593285977840424;
  target_pose.orientation.w = 0.2997700870037079;
  target_pose.orientation.x = -0.634311318397522;
  target_pose.orientation.y = 0.29266050457954407;
  target_pose.orientation.z = 0.6497206091880798;
  
  robot_arm_move_group.setPoseTarget(target_pose);
  
  auto const [success_picking_position, plan_picking_position] = [&robot_arm_move_group]{
    moveit::planning_interface::MoveGroupInterface::Plan target_pose;
    auto const ok = static_cast<bool>(robot_arm_move_group.plan(target_pose));
    return std::make_pair(ok, target_pose);
  }();
  
  if (success_picking_position) {
    robot_arm_move_group.execute(plan_picking_position);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for Moving robot arm to picking position");
  }
  
  // Open Gripper
  hand_move_group.setNamedTarget("open");
  
  moveit::planning_interface::MoveGroupInterface::Plan plan_open_gripper;
  bool success_open_gripper = (hand_move_group.plan(plan_open_gripper) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success_open_gripper) {
    hand_move_group.execute(plan_open_gripper);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for opening gripper");
  }
  
  rclcpp::sleep_for(std::chrono::seconds(3));
  
  // Close Gripper
  hand_move_group.setNamedTarget("close");
  
  moveit::planning_interface::MoveGroupInterface::Plan plan_close_gripper;
  bool success_close_gripper = (hand_move_group.plan(plan_close_gripper) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success_close_gripper) {
    hand_move_group.execute(plan_close_gripper);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for closing gripper");
  }
  
  // Approach Point
  robot_arm_move_group.setPoseTarget(approach_pose);
  
  auto const [success_approach_point2, plan_approach_point2] = [&robot_arm_move_group]{
    moveit::planning_interface::MoveGroupInterface::Plan approach_pose;
    auto const ok = static_cast<bool>(robot_arm_move_group.plan(approach_pose));
    return std::make_pair(ok, approach_pose);
  }();
  
  if (success_approach_point2) {
    robot_arm_move_group.execute(plan_approach_point2);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for Moving robot arm to approach position");
  }
  
  // Placing Approach Position
  geometry_msgs::msg::Pose placing_approach_pose;
  placing_approach_pose.position.x = 0.4687330424785614;
  placing_approach_pose.position.y = -0.0017549340846017003;
  placing_approach_pose.position.z = 0.2464482843875885;
  placing_approach_pose.orientation.w = 0.7049350738525391;
  placing_approach_pose.orientation.x = 0.0027630655094981194;
  placing_approach_pose.orientation.y = 0.7092611789703369;
  placing_approach_pose.orientation.z = -0.0027462122961878777;
  
  robot_arm_move_group.setPoseTarget(placing_approach_pose);
  
  auto const [success_placing_approach_position, plan_placing_approach_position] = [&robot_arm_move_group]{
    moveit::planning_interface::MoveGroupInterface::Plan placing_approach_pose;
    auto const ok = static_cast<bool>(robot_arm_move_group.plan(placing_approach_pose));
    return std::make_pair(ok, placing_approach_pose);
  }();
  
  if (success_placing_approach_position) {
    robot_arm_move_group.execute(plan_placing_approach_position);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for Moving robot arm to picking position");
  }
  
  // Placing Position
  geometry_msgs::msg::Pose placing_pose;
  placing_pose.position.x = 0.4687330424785614;
  placing_pose.position.y = -0.0017549340846017003;
  placing_pose.position.z = 0.1464482843875885;
  placing_pose.orientation.w = 0.7049350738525391;
  placing_pose.orientation.x = 0.0027630655094981194;
  placing_pose.orientation.y = 0.7092611789703369;
  placing_pose.orientation.z = -0.0027462122961878777;
  
  robot_arm_move_group.setPoseTarget(placing_pose);
  
  auto const [success_placing_position, plan_placing_position] = [&robot_arm_move_group]{
    moveit::planning_interface::MoveGroupInterface::Plan placing_pose;
    auto const ok = static_cast<bool>(robot_arm_move_group.plan(placing_pose));
    return std::make_pair(ok, placing_pose);
  }();
  
  if (success_placing_position) {
    robot_arm_move_group.execute(plan_placing_position);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for Moving robot arm to picking position");
  }
  
  // Open Gripper
  hand_move_group.setNamedTarget("open");
  
  moveit::planning_interface::MoveGroupInterface::Plan plan_open_gripper2;
  bool success_open_gripper2 = (hand_move_group.plan(plan_open_gripper2) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success_open_gripper2) {
    hand_move_group.execute(plan_open_gripper2);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for opening gripper");
  }
  
  rclcpp::sleep_for(std::chrono::seconds(3));
  
  // Close Gripper
  hand_move_group.setNamedTarget("close");
  
  moveit::planning_interface::MoveGroupInterface::Plan plan_close_gripper2;
  bool success_close_gripper2 = (hand_move_group.plan(plan_close_gripper2) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success_close_gripper2) {
    hand_move_group.execute(plan_close_gripper2);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for closing gripper");
  }
  
  // Placing Approach Position  
  robot_arm_move_group.setPoseTarget(placing_approach_pose);
  
  auto const [success_placing_approach_position2, plan_placing_approach_position2] = [&robot_arm_move_group]{
    moveit::planning_interface::MoveGroupInterface::Plan placing_approach_pose;
    auto const ok = static_cast<bool>(robot_arm_move_group.plan(placing_approach_pose));
    return std::make_pair(ok, placing_approach_pose);
  }();
  
  if (success_placing_approach_position2) {
    robot_arm_move_group.execute(plan_placing_approach_position2);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for Moving robot arm to picking position");
  }
  
  // home position
  robot_arm_move_group.setNamedTarget("home");
  
  moveit::planning_interface::MoveGroupInterface::Plan plan_home_position2;
  bool success_home_position2 = (robot_arm_move_group.plan(plan_home_position2) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success_home_position2) {
    robot_arm_move_group.execute(plan_home_position2);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for Moving robot arm to home position");
  }
  //move_group_interface.setPoseTarget(target_pose);
  //move_group_interface.setPositionTarget(0.35493579506874084, -0.22717544436454773, 0.27618545293807983);
  //move_group_interface.setRandomTarget();
  //move_group_interface.setNamedTarget("home");

  /*auto move_group_gripper = MoveGroupInterface(node, "hand");
  move_group_gripper.setNamedTarget("open");
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success_gripper) {
    move_group_gripper.execute(my_plan_gripper);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for Opening Gripper using named target");
  }*/
  
    rclcpp::shutdown();
    return 0;
  }
