#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <map>
 
class JointTargetListener {
public:
  JointTargetListener(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "robot_arm");
    feedback_pub_ = node_->create_publisher<std_msgs::msg::String>("robot_execution_status", 10);
 
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(5);
    move_group_->setPlannerId("RRTConnectkConfigDefault");
 
    // Gripper ActionClient
    gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
      node_, "/hand_controller/gripper_cmd"
    );
 
    // Zielpunkt-Listener
    subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "target_joint_states", 10,
      std::bind(&JointTargetListener::callback, this, std::placeholders::_1)
    );
 
    // Abbrechen-Service
    cancel_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "cancel_motion",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
 
        move_group_->stop();  // Sofort stoppen
        response->success = true;
        response->message = "Bewegung gestoppt.";
        RCLCPP_WARN(node_->get_logger(), "Bewegung wurde durch externen Befehl abgebrochen.");
      }
    );  
 
    RCLCPP_INFO(node_->get_logger(), "joint_listener gestartet und hört auf /target_joint_states");
  }
 
private:
  void callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::map<std::string, double> joint_map;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_map[msg->name[i]] = msg->position[i];
    }
 
    // Roboterbewegung
    if (joint_map.count("joint1") && joint_map.count("joint2") &&
        joint_map.count("joint3") && joint_map.count("joint4")) {
 
      std::vector<double> joints = {
        joint_map["joint1"],
        joint_map["joint2"],
        joint_map["joint3"],
        joint_map["joint4"]
      };
 
      move_group_->setJointValueTarget(joints);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = static_cast<bool>(move_group_->plan(plan));
 
      if (success) {
        RCLCPP_INFO(node_->get_logger(), "Planung erfolgreich. Führe Bewegung aus...");
        bool executed = static_cast<bool>(move_group_->execute(plan));
        if (executed) {
          RCLCPP_INFO(node_->get_logger(), "Roboter erfolgreich bewegt.");
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Bewegung konnte nicht ausgeführt werden.");
        }
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Planung fehlgeschlagen!");
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "Unvollständige Gelenkdaten für Roboter empfangen.");
    }
 
    // Gripperbewegung
    if (joint_map.count("joint_plate")) {
      double target_position = joint_map["joint_plate"];
      RCLCPP_INFO(node_->get_logger(), "Sende Greifer-Position: %.3f", target_position);
 
      if (!gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Gripper ActionServer nicht verfügbar!");
        return;
      }
 
      auto goal_msg = control_msgs::action::GripperCommand::Goal();
      goal_msg.command.position = target_position;
      goal_msg.command.max_effort = 1.0;
 
      auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
      send_goal_options.result_callback = [this](const auto & result) {
        if (result.result->reached_goal) {
          RCLCPP_INFO(node_->get_logger(), "Greifer erfolgreich bewegt.");
        } else {
          RCLCPP_WARN(node_->get_logger(), "Greiferbewegung nicht erfolgreich.");
        }
      };
 
      gripper_client_->async_send_goal(goal_msg, send_goal_options);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Kein Greiferwert (joint_plate) empfangen.");
    }
 
    // Feedback
    std::string point_name = msg->header.frame_id;
    std_msgs::msg::String feedback_msg;
    feedback_msg.data = "Ziel erreicht: " + point_name;
    feedback_pub_->publish(feedback_msg);
  }
 
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
};
 
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joint_target_listener");
 
  JointTargetListener listener(node);
 
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}