#ifndef RVIZ_TEACH_PLUGIN_HPP_
#define RVIZ_TEACH_PLUGIN_HPP_

#include <rviz_common/panel.hpp>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QLabel>
#include <QFileDialog>
#include <QTextEdit>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QListWidget>
#include <QInputDialog>
#include <QTabWidget>
#include <QSlider>
#include <QComboBox>
#include <QMessageBox>
#include <QTimer>
#include <QDebug>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include "rviz_services/srv/move_linear.hpp"
#include "rviz_services/srv/move_to_pose.hpp"
#include <rviz_common/display_context.hpp>
#include <thread>
#include <atomic>
#include <map>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rviz_teach_plugin
{

// ============================
// CustomPlugin Class Definition
// ============================

class CustomPlugin : public rviz_common::Panel
{
Q_OBJECT

public:
    // Constructor
    CustomPlugin(QWidget *parent = nullptr);

    // Destructor
    ~CustomPlugin();

    // Override from rviz_common::Panel
    void onInitialize() override;

private Q_SLOTS:
    // ============================
    // Initialization Functions
    // ============================
    void setup_joint_state_listener();
    void start_joint_state_thread();

    // ============================
    // Teach Tab Functions
    // ============================
    void onTeachPoint();
    void addWaypoint();
    void editPoint();
    void deletePoint();
    void movePointUp();
    void movePointDown();
    void saveWaypointToJson(const QString &name, const QStringList &joint_values);
    void saveJsonFile();

    // ============================
    // Move Robot Tab Functions
    // ============================
    void sendSliderJointState();
    void sendMovementRequest(bool is_x_direction, bool is_positive);
    void onMoveToHome();
    void setStepSize(int index);
    void moveToPose();  

    // ============================
    // Load File Tab Functions
    // ============================
    void openFileDialog();
    void closeFile();
    void loadWaypointDetails(const QString &waypoint_name);

    // ============================
    // Execution Functions
    // ============================
    void runAllPoints();
    void executeNextWaypoint();
    void cancelExecution();
    void moveToPoint();

    // ============================
    // Helper Functions
    // ============================
    sensor_msgs::msg::JointState::SharedPtr fetchJointStates(const std::string &node_name, int timeout_seconds);
    bool pointExists(const QString &name);
    void fetchAndPrintJointStates();
    void fetchCurrentPose();  


private:
    // ROS2 node and logger
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_ = rclcpp::get_logger("CustomPlugin");
    
    // TF2 buffer and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // UI elements
    QListWidget* point_list_;
    QLineEdit* file_path_edit_;
    QLabel* status_label_;
    QPushButton* cancel_run_button_;
    QLineEdit* x_coord_edit_;  
    QLineEdit* y_coord_edit_;  
    QLineEdit* z_coord_edit_;  

    // ROS2 publishers, subscribers, and clients
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_motion_client_;

    // Data structures for waypoints
    std::map<QString, QJsonObject> waypoint_data_;
    QStringList pending_waypoints_;
    bool is_executing_points_ = false;
    double movement_step_size_ = 10.0;

    // Joint sliders and labels
    std::map<QString, QSlider*> joint_slider_map_;
    std::map<QString, QLabel*> joint_label_map_;
    QSlider* gripper_slider_;
    QLabel* gripper_label_;

    // Service clients for linear and home movements
    rclcpp::Client<rviz_services::srv::MoveLinear>::SharedPtr move_linear_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr home_client_;
    rclcpp::Client<rviz_services::srv::MoveToPose>::SharedPtr move_to_pose_client_;
    
    // ROS2 threading and execution
    std::thread ros_thread_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
    QTimer* poll_timer_ = nullptr;

    // Background joint listener
    rclcpp::Node::SharedPtr listener_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread ros_spin_thread_;

    std::thread joint_listener_thread_;
    std::atomic<bool> running_ = true;
    rclcpp::Node::SharedPtr joint_listener_node_;

    // Add the missing timer declaration
    QTimer* slider_update_timer_;

    // Helper function to update joint state display
    void updateJointStateDisplay();
};

}  // namespace rviz_teach_plugin

#endif  // RVIZ_TEACH_PLUGIN_HPP_