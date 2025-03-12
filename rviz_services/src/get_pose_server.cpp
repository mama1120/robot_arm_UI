#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rviz_services/srv/get_pose_robot_arm.hpp"

using GetPoseRobotArm = rviz_services::srv::GetPoseRobotArm;

class RobotArmPoseService : public rclcpp::Node {
public:
    RobotArmPoseService() : Node("robot_arm_pose_service"), pose_received_(false) {
        // Create the service
        server_ = this->create_service<GetPoseRobotArm>(
            "get_robot_arm_pose",
            std::bind(&RobotArmPoseService::handle_service, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
        );

        // Subscribe to joint_states topic
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RobotArmPoseService::joint_states_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "[Server] Robot Arm Pose Service Started.");
    }

private:
    rclcpp::Service<GetPoseRobotArm>::SharedPtr server_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::vector<double> latest_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Default 7 values
    bool pose_received_; // Flag to capture pose only once

    void handle_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<GetPoseRobotArm::Request> request,
        const std::shared_ptr<GetPoseRobotArm::Response> response) 
    {
        (void)request_header;

        if (request->request_pose && pose_received_) {
            RCLCPP_INFO(this->get_logger(), "[Server] Sending latest robot arm pose.");
            for (size_t i = 0; i < 7; ++i) {
                response->pose_values[i] = latest_pose_[i];
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "[Server] No pose available yet.");
        }
    }

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!pose_received_) {
            size_t available_values = msg->position.size();
            for (size_t i = 0; i < 7; ++i) {
                if (i < available_values) {
                    latest_pose_[i] = msg->position[i];
                } else {
                    latest_pose_[i] = 0.0;  // Pad missing values with zero
                }
            }
            pose_received_ = true;
            RCLCPP_INFO(this->get_logger(), "[Server] Received robot arm pose, unsubscribing.");
            subscription_.reset(); // Unsubscribe from topic after getting the first message
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotArmPoseService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
