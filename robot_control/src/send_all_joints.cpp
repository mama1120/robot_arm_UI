#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include <sstream>
#include <unordered_map>

constexpr double RAD2DEG = 180.0 / M_PI;
constexpr double M_TO_MM = 1000.0;

class TrajectoryListener : public rclcpp::Node
{
public:
    using MoveGroup = moveit_msgs::msg::DisplayTrajectory;
    using JointState = sensor_msgs::msg::JointState;

    TrajectoryListener()
        : Node("trajectory_listener")
    {
        planned_path_subscription_ = this->create_subscription<MoveGroup>(
            "/display_planned_path", 10, 
            std::bind(&TrajectoryListener::trajectory_callback, this, std::placeholders::_1));

        joint_state_subscription_ = this->create_subscription<JointState>(
            "/joint_states", 10, 
            std::bind(&TrajectoryListener::joint_state_callback, this, std::placeholders::_1));

        // Publisher for the planned trajectory
        planned_trajectory_publisher_ = this->create_publisher<std_msgs::msg::String>("/planned_trajectory", 10);

        RCLCPP_INFO(this->get_logger(), "Waiting for a path to be planned...");
    }

private:
    void trajectory_callback(const MoveGroup::SharedPtr msg)
    {
        std::vector<std::string> planned_joint_names = msg->trajectory[0].joint_trajectory.joint_names;
        const auto& trajectory = msg->trajectory[0].joint_trajectory;

        std::vector<std::vector<double>> all_joint_positions;

        for (const auto& point : trajectory.points) {
            std::vector<double> complete_joint_positions(5, 0.0);
            
            // Fill in the joint positions from the trajectory points
            for (size_t i = 0; i < planned_joint_names.size(); ++i) {
                complete_joint_positions[i] = point.positions[i];
            }

            // For any missing joint positions (i.e., joints not in the planned trajectory),
            // use the latest joint positions from /joint_states
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                if (std::find(planned_joint_names.begin(), planned_joint_names.end(), joint_names_[i]) == planned_joint_names.end()) {
                    complete_joint_positions[i] = current_joint_states_[joint_names_[i]];
                }
            }

            all_joint_positions.push_back(complete_joint_positions);
        }

        for (size_t i = 0; i < all_joint_positions.size(); ++i) {
            print_joint_positions_in_degrees(all_joint_positions[i], "Point " + std::to_string(i));
        }

        send_all_joint_positions(all_joint_positions);

        // Publish the planned trajectory to the ROS 2 topic
        publish_planned_trajectory(all_joint_positions);
    }


    void joint_state_callback(const JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            current_joint_states_[msg->name[i]] = msg->position[i];
        }
    }


    void send_all_joint_positions(const std::vector<std::vector<double>>& all_joint_positions)
    {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            perror("Socket creation error");
            return;
        }

        struct sockaddr_in server_address;
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(12345);

        if (inet_pton(AF_INET, "10.42.0.128", &server_address.sin_addr) <= 0) {
            perror("Invalid address or address not supported");
            close(sock);
            return;
        }

        if (connect(sock, (struct sockaddr*)&server_address, sizeof(server_address)) < 0) {
            perror("Connection failed");
            close(sock);
            return;
        }

        // Prepare the data to send with a length header
        std::string data_to_send;
        for (const auto& joint_positions : all_joint_positions) {
            for (size_t i = 0; i < joint_positions.size(); ++i) {
                double value_to_send = joint_positions[i];

                // Convert joint values appropriately
                if (i == 4) {
                    value_to_send *= M_TO_MM;
                } else {
                    value_to_send *= RAD2DEG;
                }

                data_to_send += std::to_string(value_to_send);
                if (i < joint_positions.size() - 1) {
                    data_to_send += ",";
                }
            }
            data_to_send += ";"; // Add a delimiter for each set of joint positions
        }

        uint32_t data_length = htonl(data_to_send.size()); // Convert length to network byte order
        send(sock, &data_length, sizeof(data_length), 0); // Send the length header
        send(sock, data_to_send.c_str(), data_to_send.size(), 0); // Send the actual data

        char buffer[1024] = {0};
        int valread = read(sock, buffer, 1024);
        if (valread > 0) {
            std::string confirmation(buffer, valread);
            RCLCPP_INFO(this->get_logger(), "Server response: %s", confirmation.c_str());
        }

        close(sock);
    }

    void publish_planned_trajectory(const std::vector<std::vector<double>>& all_joint_positions)
    {
        std::ostringstream oss;
        for (const auto& joint_positions : all_joint_positions) {
            for (size_t i = 0; i < joint_positions.size(); ++i) {
                double value_to_publish = joint_positions[i];
                if (i == 4) {
                    value_to_publish *= M_TO_MM;
                } else {
                    value_to_publish *= RAD2DEG;
                }
                oss << value_to_publish;
                if (i < joint_positions.size() - 1) {
                    oss << ",";
                }
            }
            oss << ";";
        }

        auto message = std_msgs::msg::String();
        message.data = oss.str();
        planned_trajectory_publisher_->publish(message);
    }

    void print_joint_positions_in_degrees(const std::vector<double>& joint_positions, const std::string& prefix)
    {
        std::ostringstream joint_positions_stream;
        joint_positions_stream << prefix << " joint positions: ";
        for (size_t i = 0; i < joint_positions.size(); ++i) {
            double value_to_print = joint_positions[i];
            if (i == 4) {
                value_to_print *= M_TO_MM;
                joint_positions_stream << value_to_print << " mm";
            } else {
                value_to_print *= RAD2DEG;
                joint_positions_stream << value_to_print << " deg";
            }
            if (i < joint_positions.size() - 1) {
                joint_positions_stream << ", ";
            }
        }
        RCLCPP_INFO(this->get_logger(), "%s", joint_positions_stream.str().c_str());
    }

    rclcpp::Subscription<MoveGroup>::SharedPtr planned_path_subscription_;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planned_trajectory_publisher_;
    std::unordered_map<std::string, double> current_joint_states_;

    const std::vector<std::string> joint_names_ = {
        "joint1", "joint2", "joint3", "joint4", 
        "joint_plate"
    };

    std::unordered_map<std::string, size_t> joint_name_to_index_ = {
        {"joint1", 2}, {"joint2", 0}, {"joint3", 1}, {"joint4", 3},
        {"joint_plate", 4}
    };
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
