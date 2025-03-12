#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/set_bool.hpp>

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

class RobotArmService : public rclcpp::Node {
public:
    RobotArmService() : Node("robot_arm_service"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // Use a timer to delay the initialization of MoveGroupInterface
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // Delay of 100ms
            std::bind(&RobotArmService::initialize_move_group, this));

        // Create services for X and Z directions
        service_x_ = this->create_service<std_srvs::srv::SetBool>(
            "move_x",
            std::bind(&RobotArmService::handle_service_x, this, std::placeholders::_1, std::placeholders::_2));

        service_z_ = this->create_service<std_srvs::srv::SetBool>(
            "move_z",
            std::bind(&RobotArmService::handle_service_z, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void initialize_move_group() {
        // Create the MoveIt MoveGroup Interface
        move_group_robot_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "robot_arm");
        move_group_robot_arm_->setPlannerId("RRTConnectConfigDefault");
        move_group_robot_arm_->setPlanningTime(100.0);
        move_group_robot_arm_->setNumPlanningAttempts(10);

        // Stop the timer after initialization
        init_timer_->cancel();
    }

    // Service callback for X direction
    void handle_service_x(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        double relative_movement_x = request->data ? 0.05 : -0.05; // 5 cm in positive or negative X direction
        move_in_direction("X", relative_movement_x, response);
    }

    // Service callback for Z direction
    void handle_service_z(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        double relative_movement_z = request->data ? 0.05 : -0.05; // 5 cm in positive or negative Z direction
        move_in_direction("Z", relative_movement_z, response);
    }

    // Function to move the robot arm in a specific direction
    void move_in_direction(const std::string& direction, double distance,
                           const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (!move_group_robot_arm_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface is not initialized");
            response->success = false;
            response->message = "MoveGroupInterface is not initialized";
            return;
        }

        // Get the transform from link4_1 to base_link
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform("base_link", "link4_1", tf2::TimePointZero, tf2::durationFromSec(1.0));
            RCLCPP_INFO(this->get_logger(), "Transform from link4_1 to base_link: translation=(%.3f, %.3f, %.3f), rotation=(%.3f, %.3f, %.3f, %.3f)",
                        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z,
                        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
            response->success = false;
            response->message = "TF lookup failed";
            return;
        }

        // Create a pose in the link4_1 frame with the relative movement
        geometry_msgs::msg::PoseStamped pose_link4_1;
        pose_link4_1.header.frame_id = "link4_1";
        pose_link4_1.pose.position.x = (direction == "X") ? distance : 0.0;
        pose_link4_1.pose.position.y = 0.0;
        pose_link4_1.pose.position.z = (direction == "Z") ? distance : 0.0;
        pose_link4_1.pose.orientation.w = 1.0; // No rotation

        RCLCPP_INFO(this->get_logger(), "Pose in link4_1 (after adding relative movement): x=%.3f, y=%.3f, z=%.3f",
                    pose_link4_1.pose.position.x, pose_link4_1.pose.position.y, pose_link4_1.pose.position.z);

        // Transform the pose back to the base_link frame
        geometry_msgs::msg::PoseStamped pose_base_link;
        try {
            pose_base_link = tf_buffer_.transform(pose_link4_1, "base_link", tf2::durationFromSec(1.0));
            RCLCPP_INFO(this->get_logger(), "Pose in base_link (after transformation back): x=%.3f, y=%.3f, z=%.3f",
                        pose_base_link.pose.position.x, pose_base_link.pose.position.y, pose_base_link.pose.position.z);
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "TF transformation failed: %s", ex.what());
            response->success = false;
            response->message = "TF transformation failed";
            return;
        }

        // Move the robot arm to the new pose
        RCLCPP_INFO(this->get_logger(), "Moving to new pose: x=%.3f, y=%.3f, z=%.3f",
                    pose_base_link.pose.position.x, pose_base_link.pose.position.y, pose_base_link.pose.position.z);
        move_to_position(*move_group_robot_arm_, pose_base_link.pose, "pose_setter");

        response->success = true;
        response->message = "Movement successful";
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_robot_arm_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_x_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_z_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotArmService>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}