#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <iostream>

class JointPositionReceiver : public rclcpp::Node
{
public:
    JointPositionReceiver()
        : Node("joint_position_receiver")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/current_joint_positions", 10);

        const char *SERVER_IP = "0.0.0.0"; // Listen on all available interfaces
        int SERVER_PORT = 12346;           // Port for receiving feedback

        // Create TCP socket
        server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            rclcpp::shutdown();
        }

        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
        server_addr.sin_port = htons(SERVER_PORT);

        if (bind(server_socket_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Bind failed");
            close(server_socket_);
            rclcpp::shutdown();
        }

        if (listen(server_socket_, 1) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to listen on socket");
            close(server_socket_);
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "Listening for feedback on %s:%d", SERVER_IP, SERVER_PORT);

        client_socket_ = accept(server_socket_, (struct sockaddr *)&client_addr_, &client_len_);
        if (client_socket_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to accept connection");
            close(server_socket_);
            rclcpp::shutdown();
        }

        // Timer to receive and process data periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&JointPositionReceiver::receive_and_publish, this));
    }

    ~JointPositionReceiver()
    {
        close(client_socket_);
        close(server_socket_);
    }

private:
    void receive_and_publish()
    {
        // Step 1: Read the length header (4 bytes)
        uint32_t length_header = 0;
        int bytes_received = recv(client_socket_, &length_header, sizeof(length_header), 0);
        if (bytes_received != sizeof(length_header))
        {
            if (bytes_received == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Client disconnected. Waiting for new connection...");
                client_socket_ = accept(server_socket_, (struct sockaddr *)&client_addr_, &client_len_);
            }
            else if (bytes_received < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            {
                RCLCPP_ERROR(this->get_logger(), "Error receiving length header: %s", strerror(errno));
            }
            return;
        }

        // Convert the length from network byte order to host byte order
        length_header = ntohl(length_header);

        // Step 2: Read the actual message using the length we just received
        std::vector<char> buffer(length_header + 1, 0); // +1 for null terminator
        bytes_received = recv(client_socket_, buffer.data(), length_header, 0);

        if (bytes_received == length_header)
        {
            buffer[length_header] = '\0'; // Null-terminate the string
            std::string received_positions(buffer.data());

            RCLCPP_INFO(this->get_logger(), "Received current joint positions: %s", received_positions.c_str());

            // Publish the received joint positions to a ROS topic
            auto message = std_msgs::msg::String();
            message.data = received_positions;
            publisher_->publish(message);
        }
        else if (bytes_received < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", strerror(errno));
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int server_socket_;
    int client_socket_;
    struct sockaddr_in client_addr_;
    socklen_t client_len_ = sizeof(client_addr_);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPositionReceiver>());
    rclcpp::shutdown();
    return 0;
}
