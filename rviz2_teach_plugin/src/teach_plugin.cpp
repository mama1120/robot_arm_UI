#include "rviz2_teach_plugin/teach_plugin.hpp"
#include <chrono>
#include <random>

namespace rviz_teach_plugin
{

}  // namespace rviz_teach_plugin

// ============================
// Implementation
// ============================

// Constructor
rviz_teach_plugin::CustomPlugin::CustomPlugin(QWidget *parent)
    : rviz_common::Panel(parent)
{
    // Initialize ROS2 node
    node_ = std::make_shared<rclcpp::Node>("rviz_teach_plugin");

    // Initialize publishers and clients
    pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("move_robot", 10);
    joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("target_joint_states", 10);
    cancel_motion_client_ = node_->create_client<std_srvs::srv::Trigger>("cancel_motion");
     
    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    // Start joint state listener
    setup_joint_state_listener();

    // Create service clients
    move_linear_client_ = node_->create_client<rviz_services::srv::MoveLinear>("move_linear");
    home_client_ = node_->create_client<std_srvs::srv::SetBool>("move_to_home");
    move_to_pose_client_ = node_->create_client<rviz_services::srv::MoveToPose>("move_to_pose");

    // Create main UI layout
    auto *tab_widget = new QTabWidget;
    auto *main_layout = new QVBoxLayout;

    // ============================
    // Teach Tab 
    // ============================

    QWidget *teach_tab = new QWidget;
    QVBoxLayout *teach_layout = new QVBoxLayout;
    point_list_ = new QListWidget;
    teach_layout->addWidget(point_list_);

    // Add buttons for editing, saving, and deleting points
    auto *edit_button = new QPushButton("Edit Point Name");
    teach_layout->addWidget(edit_button);
    auto *save_delete_layout = new QHBoxLayout;
    auto *save_button = new QPushButton("Save JSON");
    auto *delete_point_button = new QPushButton("Delete Point");
    save_delete_layout->addWidget(save_button);
    save_delete_layout->addWidget(delete_point_button);
    teach_layout->addLayout(save_delete_layout);

    // Add buttons for moving points up and down
    auto *move_buttons_layout = new QHBoxLayout;
    auto *move_point_up_button = new QPushButton("Move Point Up");
    auto *move_point_down_button = new QPushButton("Move Point Down");
    move_buttons_layout->addWidget(move_point_up_button);
    move_buttons_layout->addWidget(move_point_down_button);
    teach_layout->addLayout(move_buttons_layout);

    // Add button for teaching a new point
    auto *teach_button = new QPushButton("Teach Point");
    teach_layout->addWidget(teach_button);

    // Add buttons for moving to a point and running all points
    auto *move_run_buttons_layout = new QVBoxLayout;
    auto *button_row = new QHBoxLayout;
    auto *move_to_point_button = new QPushButton("Move to Point");
    auto *run_all_points_button = new QPushButton("Run all Points");
    button_row->addWidget(move_to_point_button);
    button_row->addWidget(run_all_points_button);
    move_run_buttons_layout->addLayout(button_row);

    // Add status label and cancel button
    status_label_ = new QLabel();
    status_label_->setVisible(false);
    status_label_->setStyleSheet("font-weight: bold; color: orange;");
    move_run_buttons_layout->addWidget(status_label_);
    cancel_run_button_ = new QPushButton("Cancel");
    cancel_run_button_->setVisible(false);
    button_row->addWidget(cancel_run_button_);
    teach_layout->addLayout(move_run_buttons_layout);
    teach_tab->setLayout(teach_layout);
    tab_widget->addTab(teach_tab, "Teach");
 
    // ============================
    // Move Robot Tab
    // ============================

    QWidget *move_tab = new QWidget;
    QVBoxLayout *move_layout = new QVBoxLayout;

    // Add joint sliders
    auto *joint_control_label = new QLabel("Joint Controls:");
    move_layout->addWidget(joint_control_label);

    const QMap<int, QPair<int, int>> joint_ranges = {
        {1, {-150, 150}},
        {2, {0, 95}},
        {3, {-115, 120}},
        {4, {-90, 100}}
    };

    for (int i = 1; i <= 4; ++i) {
        QString joint_name = QString("joint%1").arg(i);
        auto *joint_layout = new QHBoxLayout;
        auto *joint_label = new QLabel(QString("Joint %1 Angle:").arg(i));
        auto *joint_value = new QLabel("0°");
        joint_value->setFixedWidth(50);
        joint_value->setObjectName(joint_name + "_label");
        auto *joint_slider = new QSlider(Qt::Horizontal);
        joint_slider->setRange(joint_ranges[i].first, joint_ranges[i].second);
        joint_slider->setValue(0);
        joint_slider->setObjectName(joint_name);
        
        // Improve slider responsiveness
        joint_slider->setTickPosition(QSlider::TicksBelow);
        joint_slider->setTickInterval(10);
        joint_slider->setPageStep(5);
        joint_slider->setSingleStep(1);
        
        joint_slider_map_[joint_name] = joint_slider;
        joint_label_map_[joint_name] = joint_value;
        joint_layout->addWidget(joint_label);
        joint_layout->addWidget(joint_slider);
        joint_layout->addWidget(joint_value);
        move_layout->addLayout(joint_layout);

        // Connect slider signals for better responsiveness
        connect(joint_slider, &QSlider::valueChanged, this, [this, joint_value](int value) {
            // Update label immediately for visual feedback
            joint_value->setText(QString::number(value) + "°");
            // Debounce the actual joint command
            slider_update_timer_->start();
        });
        
        // Keep sliderReleased for immediate response when user stops dragging
        connect(joint_slider, &QSlider::sliderReleased, this, [this]() {
            slider_update_timer_->stop();
            sendSliderJointState();
        });
    }

    // Add gripper slider with same improvements
    auto *gripper_label = new QLabel("Gripper Opening:");
    auto *gripper_slider = new QSlider(Qt::Horizontal);
    gripper_slider->setRange(0, 100);
    gripper_slider->setValue(0);
    gripper_slider->setObjectName("gripper");
    
    // Improve gripper slider responsiveness
    gripper_slider->setTickPosition(QSlider::TicksBelow);
    gripper_slider->setTickInterval(10);
    gripper_slider->setPageStep(5);
    gripper_slider->setSingleStep(1);
    
    auto *gripper_value = new QLabel("0%");
    gripper_value->setFixedWidth(50);
    gripper_value->setObjectName("gripper_label");
    gripper_slider_ = gripper_slider;
    gripper_label_ = gripper_value;
    joint_slider_map_["joint_plate"] = gripper_slider_;
    joint_label_map_["joint_plate"] = gripper_label_;
    auto *gripper_layout = new QHBoxLayout;
    gripper_layout->addWidget(gripper_label);
    gripper_layout->addWidget(gripper_slider);
    gripper_layout->addWidget(gripper_value);
    move_layout->addLayout(gripper_layout);

    connect(gripper_slider, &QSlider::valueChanged, this, [this, gripper_value](int value) {
        // Update label immediately for visual feedback
        gripper_value->setText(QString::number(value) + "%");
        // Debounce the actual joint command
        slider_update_timer_->start();
    });
    
    connect(gripper_slider, &QSlider::sliderReleased, this, [this]() {
        slider_update_timer_->stop();
        sendSliderJointState();
    });

    // Add coordinate input fields for move_to_pose in a single row
    auto *coordinate_layout = new QHBoxLayout;
    
    // X coordinate
    x_coord_edit_ = new QLineEdit;
    x_coord_edit_->setPlaceholderText("X");
    
    // Y coordinate
    y_coord_edit_ = new QLineEdit;
    y_coord_edit_->setPlaceholderText("Y");
    
    // Z coordinate
    z_coord_edit_ = new QLineEdit;
    z_coord_edit_->setPlaceholderText("Z");
    
    // Move to pose button
    auto *move_to_pose_button = new QPushButton("Move to Pos");
    
    // Get current position button
    auto *get_position_button = new QPushButton("Get Pos");
    
    // Add all elements to the horizontal layout
    coordinate_layout->addWidget(x_coord_edit_);
    coordinate_layout->addWidget(y_coord_edit_);
    coordinate_layout->addWidget(z_coord_edit_);
    coordinate_layout->addWidget(move_to_pose_button);
    coordinate_layout->addWidget(get_position_button);
    // Add a descriptive label before the layout
    auto *coordinate_label = new QLabel("Move to global Position (m):");
    move_layout->addWidget(coordinate_label);
    move_layout->addLayout(coordinate_layout);

    // Add linear movement controls
    auto *lin_movement_label = new QLabel("Linear Movements:");
    move_layout->addWidget(lin_movement_label);
    auto *step_size_layout = new QHBoxLayout;
    auto *step_size_label = new QLabel("Step Size:");
    auto *step_size_dropdown = new QComboBox;
    step_size_dropdown->addItem("1mm", 1.0);
    step_size_dropdown->addItem("5mm", 5.0);
    step_size_dropdown->addItem("10mm", 10.0);
    step_size_dropdown->addItem("20mm", 20.0);
    step_size_dropdown->addItem("50mm", 50.0);
    step_size_dropdown->setCurrentIndex(2);
    step_size_layout->addWidget(step_size_label);
    step_size_layout->addWidget(step_size_dropdown);
    move_layout->addLayout(step_size_layout);

    auto *lin_layout = new QHBoxLayout;
    auto *move_x_button = new QPushButton("X+");
    auto *move_x_neg_button = new QPushButton("X-");
    auto *move_z_button = new QPushButton("Z+");
    auto *move_z_neg_button = new QPushButton("Z-");
    auto *home_button = new QPushButton("Home");
    auto *zero_button = new QPushButton("Zero");

    lin_layout->addWidget(move_x_button);
    lin_layout->addWidget(move_x_neg_button);
    lin_layout->addWidget(move_z_button);
    lin_layout->addWidget(move_z_neg_button);
    lin_layout->addWidget(home_button);
    lin_layout->addWidget(zero_button);
    move_layout->addLayout(lin_layout);

    // Add waypoint button
    auto *add_point_button = new QPushButton("Add Point");
    move_layout->addWidget(add_point_button);

    move_tab->setLayout(move_layout);
    tab_widget->addTab(move_tab, "Move Robot");

    // ============================
    // Load File Tab
    // ============================

    QWidget *load_tab = new QWidget;
    QVBoxLayout *load_layout = new QVBoxLayout;
    file_path_edit_ = new QLineEdit;
    file_path_edit_->setReadOnly(true);
    file_path_edit_->setPlaceholderText("Select JSON file...");
    auto *browse_button = new QPushButton("Browse");
    auto *close_file_button = new QPushButton("Close File");
    load_layout->addWidget(file_path_edit_);
    load_layout->addWidget(browse_button);
    load_layout->addWidget(close_file_button);
    load_tab->setLayout(load_layout);
    tab_widget->addTab(load_tab, "Load File");

    // Main layout
    main_layout->addWidget(tab_widget);
    setLayout(main_layout);

    // Connect buttons to their respective slots
    connect(save_button, &QPushButton::clicked, this, &CustomPlugin::saveJsonFile);
    connect(edit_button, &QPushButton::clicked, this, &CustomPlugin::editPoint);
    connect(teach_button, &QPushButton::clicked, this, &CustomPlugin::onTeachPoint);
    connect(browse_button, &QPushButton::clicked, this, &CustomPlugin::openFileDialog);
    connect(close_file_button, &QPushButton::clicked, this, &CustomPlugin::closeFile);
    connect(move_to_point_button, &QPushButton::clicked, this, &CustomPlugin::moveToPoint);
    connect(run_all_points_button, &QPushButton::clicked, this, &CustomPlugin::runAllPoints);
    connect(add_point_button, &QPushButton::clicked, this, &CustomPlugin::addWaypoint);
    connect(delete_point_button, &QPushButton::clicked, this, &CustomPlugin::deletePoint);
    connect(move_point_up_button, &QPushButton::clicked, this, &CustomPlugin::movePointUp);
    connect(move_point_down_button, &QPushButton::clicked, this, &CustomPlugin::movePointDown);
    connect(step_size_dropdown, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CustomPlugin::setStepSize);

    // Connect movement buttons
    connect(move_x_button, &QPushButton::clicked, this, [this]() { sendMovementRequest(true, true); });
    connect(move_x_neg_button, &QPushButton::clicked, this, [this]() { sendMovementRequest(true, false); });
    connect(move_z_button, &QPushButton::clicked, this, [this]() { sendMovementRequest(false, true); });
    connect(move_z_neg_button, &QPushButton::clicked, this, [this]() { sendMovementRequest(false, false); });
    connect(home_button, &QPushButton::clicked, this, &CustomPlugin::onMoveToHome);
    connect(cancel_run_button_, &QPushButton::clicked, this, &CustomPlugin::cancelExecution);
    connect(move_to_pose_button, &QPushButton::clicked, this, &CustomPlugin::moveToPose);
    connect(get_position_button, &QPushButton::clicked, this, &CustomPlugin::fetchCurrentPose);
    connect(zero_button, &QPushButton::clicked, this, [this]() {
        // Set all joint sliders to zero
        for (const auto& pair : joint_slider_map_) {
            QSlider* slider = pair.second;
            if (slider) {
                slider->blockSignals(true);
                slider->setValue(0);
                slider->blockSignals(false);
            }
        }

        // Update all labels to show zero values
        for (const auto& pair : joint_label_map_) {
            QString joint_name = pair.first;
            QLabel* label = pair.second;
            if (label) {
                if (joint_name == "joint_plate") {
                    label->setText("0%");
                } else {
                    label->setText("0°");
                }
            }
        }

        // Create and publish a JointState message with all joints set to zero
        sensor_msgs::msg::JointState msg;
        msg.name = {"joint2", "joint3", "joint1", "joint4", "joint_plate"};
        msg.position = {0.0, 0.0, 0.0, 0.0, 0.0}; // All joints to zero
        msg.header.stamp = node_->now();
        msg.header.frame_id = "zero_command";
        
        // Publish the message
        joint_pub_->publish(msg);

        RCLCPP_INFO(logger_, "All joints set to zero position");
    });

    // Add slider update timer for debouncing
    slider_update_timer_ = new QTimer(this);
    slider_update_timer_->setSingleShot(true);
    slider_update_timer_->setInterval(100); // 100ms debounce
    connect(slider_update_timer_, &QTimer::timeout, this, &CustomPlugin::sendSliderJointState);

    // Start the ROS2 node in a separate thread
    ros_thread_ = std::thread([this]() { rclcpp::spin(node_); });

    // Subscribe to robot execution status
    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "robot_execution_status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(logger_, "Feedback from robot: %s", msg->data.c_str());
            if (is_executing_points_) {
                executeNextWaypoint();
            }
        });

    // Timer for polling joint states
    poll_timer_ = new QTimer(this);
    connect(poll_timer_, &QTimer::timeout, this, [this]() {
        updateJointStateDisplay();
    });
    poll_timer_->start(1000);  // Reduce to 1 second to avoid conflicts
}

// Destructor
rviz_teach_plugin::CustomPlugin::~CustomPlugin()
{
    rclcpp::shutdown();
    if (ros_thread_.joinable()) {
        ros_thread_.join();
    }
    running_ = false;
    if (joint_listener_thread_.joinable())
        joint_listener_thread_.join();

    poll_timer_->stop();
}

// ============================
// Initialization Functions
// ============================

void rviz_teach_plugin::CustomPlugin::onInitialize()
{
    // Initialize the ROS2 node for this plugin
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    // Subscribe to the "/joint_states" topic to receive joint state updates
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            // Store the latest joint state message
            latest_joint_state_ = msg;
        });
}

void rviz_teach_plugin::CustomPlugin::setup_joint_state_listener()
{
    // Create a dedicated ROS2 node for joint state listening
    joint_listener_node_ = std::make_shared<rclcpp::Node>("joint_listener_node");

    // Create a subscription to the "/joint_states" topic with SensorDataQoS
    joint_listener_node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
            latest_joint_state_ = msg;
        });

    // Create an executor and spin thread for the listener node
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(joint_listener_node_);
    ros_spin_thread_ = std::thread([this]() {
        executor_->spin();
    });

    // Timer for periodic joint state output (runs in the Qt thread)
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
        if (!latest_joint_state_) return;

        /*qDebug() << "==== Current Joint States ====";
        for (size_t i = 0; i < latest_joint_state_->name.size(); ++i) {
            QString name = QString::fromStdString(latest_joint_state_->name[i]);
            double pos = latest_joint_state_->position[i];
            qDebug() << name << ": " << pos;
        }
            */
    });
    timer->start(500); // Poll every 500ms
}

void rviz_teach_plugin::CustomPlugin::start_joint_state_thread()
{
    // Start a background thread to listen for joint state updates
    joint_listener_thread_ = std::thread([this]() {
        listener_node_ = std::make_shared<rclcpp::Node>("joint_listener_background");

        // Subscribe to the "/joint_states" topic
        auto sub = listener_node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            [](const sensor_msgs::msg::JointState::SharedPtr msg) {
                // Print the joint states to the debug console
                //qDebug() << "=== Live JointStates ===";
                for (size_t i = 0; i < msg->name.size(); ++i) {
                    qDebug() << QString::fromStdString(msg->name[i]) << ": " << msg->position[i];
                }
            });

        // Create a single-threaded executor to spin the node
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(listener_node_);

        // Spin the executor while the application is running
        while (rclcpp::ok() && running_) {
            exec.spin_once();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
}

// ============================
// Teach Tab Functions
// ============================

void rviz_teach_plugin::CustomPlugin::onTeachPoint()
{
    QListWidgetItem *selectedItem = point_list_->currentItem();
    if (!selectedItem) {
        RCLCPP_WARN(logger_, "No point selected!");
        return;
    }

    QString waypoint_name = selectedItem->text();
    if (waypoint_data_.find(waypoint_name) != waypoint_data_.end()) {
        // Fetch joint states using the helper function
        auto msg = fetchJointStates("joint_state_reader_temp", 2);
        if (!msg) {
            RCLCPP_WARN(logger_, "Failed to fetch joint states!");
            return;
        }

        // Map joint names to their positions
        std::map<std::string, double> joint_map;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            joint_map[msg->name[i]] = msg->position[i];
        }

        // Update waypoint data
        waypoint_data_[waypoint_name]["joint1"] = joint_map["joint1"];
        waypoint_data_[waypoint_name]["joint2"] = joint_map["joint2"];
        waypoint_data_[waypoint_name]["joint3"] = joint_map["joint3"];
        waypoint_data_[waypoint_name]["joint4"] = joint_map["joint4"];
        waypoint_data_[waypoint_name]["gripper"] = joint_map["joint_plate"];
    }

    RCLCPP_INFO(logger_, "Updated values for %s", waypoint_name.toStdString().c_str());

}

void rviz_teach_plugin::CustomPlugin::addWaypoint()
{
    QString base_name = "Waypoint";
    int index = 1;
    QString waypoint_name;

    // Generate a unique waypoint name
    do {
        waypoint_name = base_name + "_" + QString::number(index);
        index++;
    } while (pointExists(waypoint_name));

    // Fetch joint states using the helper function
    auto msg = fetchJointStates("joint_state_reader_temp", 2);
    if (!msg) {
        RCLCPP_WARN(logger_, "Failed to fetch joint states!");
        return;
    }

    // Map joint names to their positions
    std::map<std::string, double> joint_map;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        joint_map[msg->name[i]] = msg->position[i];
    }

    // Save the new waypoint
    QJsonObject new_point;
    new_point["name"] = waypoint_name;
    new_point["joint1"] = joint_map["joint1"];
    new_point["joint2"] = joint_map["joint2"];
    new_point["joint3"] = joint_map["joint3"];
    new_point["joint4"] = joint_map["joint4"];
    new_point["gripper"] = joint_map["joint_plate"];

    // Add the waypoint to the list and data map
    point_list_->addItem(waypoint_name);
    waypoint_data_[waypoint_name] = new_point;

    RCLCPP_INFO(logger_, "New waypoint saved: %s", waypoint_name.toStdString().c_str());
}


// Save a waypoint to a JSON file
void saveWaypointToJson(const QString &name, const QStringList &joint_values)
{
    QString file_name = "waypoints.json";
    QFile file(file_name);
    QJsonArray points;

    // Load existing waypoints if the file exists
    if (file.exists() && file.open(QIODevice::ReadOnly)) {
        QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
        file.close();
        if (doc.isObject() && doc.object().contains("points")) {
        points = doc.object()["points"].toArray();
        }
    }

    // Create a new waypoint object
    QJsonObject new_point;
    new_point["name"] = name;
    new_point["joint1"] = joint_values[0].toDouble();
    new_point["joint2"] = joint_values[1].toDouble();
    new_point["joint3"] = joint_values[2].toDouble();
    new_point["joint4"] = joint_values[3].toDouble();
    new_point["gripper"] = joint_values[4].toDouble();
    points.append(new_point);

    // Save the updated waypoints to the file
    QJsonObject root;
    root["points"] = points;
    file.open(QIODevice::WriteOnly);
    file.write(QJsonDocument(root).toJson());
    file.close();

    RCLCPP_INFO(rclcpp::get_logger("CustomPlugin"), "New waypoint '%s' saved!", name.toStdString().c_str());
}

void rviz_teach_plugin::CustomPlugin::editPoint()
{
    // Get the currently selected item in the point list
    QListWidgetItem *selectedItem = point_list_->currentItem();
    if (selectedItem)
    {
        // Retrieve the old name of the selected point
        QString oldName = selectedItem->text();
        bool ok;

        // Prompt the user to enter a new name for the point
        QString newName = QInputDialog::getText(nullptr, "Edit Point", "Enter new point name:", QLineEdit::Normal, oldName, &ok);

        // Check if the user confirmed the input, the new name is not empty, 
        // it is different from the old name, and it does not already exist
        if (ok && !newName.isEmpty() && newName != oldName && !pointExists(newName))
        {
            // Rename the point in the list
            selectedItem->setText(newName);

            // Rename the associated data in the waypoint map
            auto it = waypoint_data_.find(oldName);
            if (it != waypoint_data_.end()) {
                QJsonObject pointData = it->second;
                pointData["name"] = newName;  // Update the name in the data
                waypoint_data_.erase(it);    // Remove the old entry
                waypoint_data_[newName] = pointData;  // Add the updated entry with the new name
            }
        }
    }
}

void rviz_teach_plugin::CustomPlugin::deletePoint()
{
    // Get the currently selected item in the point list
    QListWidgetItem *selectedItem = point_list_->currentItem();
    if (selectedItem) {
        // Retrieve the name of the selected point
        QString waypoint_name = selectedItem->text();

        // Remove the point from the waypoint data map
        waypoint_data_.erase(waypoint_name);

        // Remove the point from the list widget
        delete point_list_->takeItem(point_list_->row(selectedItem));
    }
}

void rviz_teach_plugin::CustomPlugin::movePointUp()
{
    // Get the current index of the selected item in the point list
    int currentIndex = point_list_->currentRow();
    if (currentIndex > 0) {
        // Remove the item from its current position
        QListWidgetItem *item = point_list_->takeItem(currentIndex);

        // Insert the item at the position above
        point_list_->insertItem(currentIndex - 1, item);

        // Set the moved item as the currently selected item
        point_list_->setCurrentItem(item);
    }
}

void rviz_teach_plugin::CustomPlugin::movePointDown()
{
    // Get the current index of the selected item in the point list
    int currentIndex = point_list_->currentRow();
    if (currentIndex >= 0 && currentIndex < point_list_->count() - 1) {
        // Remove the item from its current position
        QListWidgetItem *item = point_list_->takeItem(currentIndex);

        // Insert the item at the position below
        point_list_->insertItem(currentIndex + 1, item);

        // Set the moved item as the currently selected item
        point_list_->setCurrentItem(item);
    }
}

void rviz_teach_plugin::CustomPlugin::saveWaypointToJson(const QString &name, const QStringList &joint_values)
{
    QString file_name = "waypoints.json";
    QFile file(file_name);
    QJsonArray points;

    // Load existing waypoints if the file exists
    if (file.exists() && file.open(QIODevice::ReadOnly)) {
        QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
        file.close();
        if (doc.isObject() && doc.object().contains("points")) {
        points = doc.object()["points"].toArray();
        }
    }

    // Create a new waypoint object
    QJsonObject new_point;
    new_point["name"] = name;
    new_point["joint1"] = joint_values[0].toDouble();
    new_point["joint2"] = joint_values[1].toDouble();
    new_point["joint3"] = joint_values[2].toDouble();
    new_point["joint4"] = joint_values[3].toDouble();
    new_point["gripper"] = joint_values[4].toDouble();
    points.append(new_point);

    // Save the updated waypoints to the file
    QJsonObject root;
    root["points"] = points;
    file.open(QIODevice::WriteOnly);
    file.write(QJsonDocument(root).toJson());
    file.close();

    RCLCPP_INFO(rclcpp::get_logger("CustomPlugin"), "New waypoint '%s' saved!", name.toStdString().c_str());
}

void rviz_teach_plugin::CustomPlugin::saveJsonFile()
{
    // Check if there are any points to save
    if (point_list_->count() == 0) {
        RCLCPP_WARN(logger_, "No points available to save.");
        return;
    }

    // Open a file dialog to select a location to save the JSON file
    QString file_name = QFileDialog::getSaveFileName(this, "Save JSON File", "", "JSON Files (*.json)");
    if (file_name.isEmpty()) {
        // If no file is selected, return early
        return;
    }

    // Open the selected file for writing
    QFile file(file_name);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        // Log an error if the file cannot be opened
        RCLCPP_ERROR(logger_, "Failed to save file: %s", file_name.toStdString().c_str());
        return;
    }

    // Create a JSON array to store the points
    QJsonArray points;
    for (int i = 0; i < point_list_->count(); ++i) {
        // Retrieve the name of each point
        QString name = point_list_->item(i)->text();
        if (waypoint_data_.find(name) != waypoint_data_.end()) {
            // Add the point data to the JSON array
            points.append(waypoint_data_[name]);
        }
    }

    // Create a root JSON object and add the points array
    QJsonObject obj;
    obj["points"] = points;

    // Write the JSON object to the file
    QJsonDocument doc(obj);
    file.write(doc.toJson());
    file.close();
}

// ============================
// Move Robot Tab Functions
// ============================

void rviz_teach_plugin::CustomPlugin::sendSliderJointState()
{
    // Create a JointState message to publish the slider values
    sensor_msgs::msg::JointState msg;
    msg.name = {"joint2", "joint3", "joint1", "joint4", "joint_plate"}; // Joint names

    std::map<std::string, double> joint_values;

    // Iterate through the joint sliders and retrieve their values
    for (const auto& pair : joint_slider_map_) {
        std::string name = pair.first.toStdString();
        QSlider* slider = pair.second;
        double value = slider->value();

        // Special handling for the gripper (convert percentage to millimeters)
        if (name == "joint_plate") {
            joint_values[name] = value / 100.0 * 0.01;
        } else {
            joint_values[name] = value * M_PI / 180.0; // Convert degrees to radians
        }
    }

    // Assign the joint positions to the message
    msg.position = {
        joint_values["joint2"],
        joint_values["joint3"],
        joint_values["joint1"],
        joint_values["joint4"],
        joint_values["joint_plate"]
    };

    // Set the message header
    msg.header.stamp = node_->now();
    msg.header.frame_id = "slider_command";

    // Publish the message
    joint_pub_->publish(msg);

    RCLCPP_INFO(logger_, "Slider values sent.");
}

void rviz_teach_plugin::CustomPlugin::sendMovementRequest(bool is_x_direction, bool is_positive)
{
    // Wait for the 'move_linear' service to be available (timeout after 2 seconds)
    if (!move_linear_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(logger_, "Service move_linear not available");
        
        // Display error message to the user
        QMessageBox::warning(
            this,
            "Service Unavailable",
            "The move_linear service is not available. Please check if the robot control node is running."
        );
        
        return;
    }

    // Create a new service request
    auto request = std::make_shared<rviz_services::srv::MoveLinear::Request>();

    // Set the direction string based on the boolean input
    // "X" for horizontal, "Z" for vertical movement
    request->direction = is_x_direction ? "X" : "Z";

    // Set the distance to move in millimeters
    // It will be positive or negative based on `is_positive`
    request->distance_mm = movement_step_size_ * (is_positive ? 1.0 : -1.0);

    // Create direction description for user messages
    QString directionDesc = QString("%1%2").arg(
        is_x_direction ? "X" : "Z",
        is_positive ? "+" : "-");

    // Send the service request asynchronously
    auto future = move_linear_client_->async_send_request(
        request,
        // Lambda callback to handle the service response
        [this, direction = request->direction, directionDesc](rclcpp::Client<rviz_services::srv::MoveLinear>::SharedFuture response) {
            if (response.get()->success) {
                // Log success message
                RCLCPP_INFO(logger_, "Move %s successful", direction.c_str());
            } else {
                // Log error message
                RCLCPP_ERROR(logger_, "Move %s failed", direction.c_str());
                
                // Show a warning message box in the Qt thread
                QMetaObject::invokeMethod(this, [this, directionDesc]() {
                    QMessageBox::warning(
                        this,
                        "Movement Not Possible",
                        QString("Cannot move in %1 direction. The robot has probably reached its kinematic limits in this direction.")
                            .arg(directionDesc)
                    );
                }, Qt::QueuedConnection);
            }
        }
    );
}
void rviz_teach_plugin::CustomPlugin::onMoveToHome()
{
    // Wait for the 'move_to_home' service to be available (timeout after 2 seconds)
    if (!home_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(logger_, "Service /move_to_home not available");
        
        // Display error message to the user
        QMessageBox::warning(
            this,
            "Service Unavailable",
            "The move_to_home service is not available. Please check if the robot control node is running."
        );
        
        return;
    }

    // Create a new service request
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    // Set the request to true, assuming this triggers the "move to home" behavior
    request->data = true;

    // Send the request asynchronously
    home_client_->async_send_request(
        request,
        // Lambda callback to handle the service response
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response) {
            if (response.get()->success) {
                // Log success message with additional info from the response
                RCLCPP_INFO(logger_, "Move to home successful: %s", response.get()->message.c_str());
            } else {
                // Log error message with details
                RCLCPP_ERROR(logger_, "Move to home failed: %s", response.get()->message.c_str());
                
                // Show a warning message box in the Qt thread with the error message from the service
                QMetaObject::invokeMethod(this, [this, message = response.get()->message]() {
                    QMessageBox::warning(
                        this,
                        "Home Movement Failed",
                        QString("Failed to move robot to home position: %1")
                            .arg(QString::fromStdString(message))
                    );
                }, Qt::QueuedConnection);
            }
        }
    );
}

void rviz_teach_plugin::CustomPlugin::setStepSize(int index)
{
    // Get the step size dropdown that triggered this function
    QComboBox *step_size_dropdown = qobject_cast<QComboBox*>(sender());
    if (step_size_dropdown) {
        // Update the movement step size based on the selected index
        movement_step_size_ = step_size_dropdown->itemData(index).toDouble();
        RCLCPP_INFO(logger_, "Step size changed to %f mm", movement_step_size_);
    }
}

void rviz_teach_plugin::CustomPlugin::moveToPose()
{
    // Wait for the service to be available
    if (!move_to_pose_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(logger_, "Service move_to_pose not available");
        
        QMessageBox::warning(
            this,
            "Service Unavailable",
            "The move_to_pose service is not available. Please check if the robot control node is running."
        );
        
        return;
    }

    // Get coordinate values from input fields
    bool x_ok = false, y_ok = false, z_ok = false;
    double x = x_coord_edit_->text().toDouble(&x_ok);
    double y = y_coord_edit_->text().toDouble(&y_ok);
    double z = z_coord_edit_->text().toDouble(&z_ok);

    // Validate input
    if (!x_ok || !y_ok || !z_ok) {
        QMessageBox::warning(
            this,
            "Invalid Input",
            "Please enter valid numeric values for X, Y, and Z coordinates."
        );
        return;
    }

    // Create a service request
    auto request = std::make_shared<rviz_services::srv::MoveToPose::Request>();
    request->x = x;
    request->y = y;
    request->z = z;

    RCLCPP_INFO(logger_, "Sending move_to_pose request: X=%.2f, Y=%.2f, Z=%.2f", x, y, z);

    // Send the service request asynchronously
    auto future = move_to_pose_client_->async_send_request(
        request,
        [this, x, y, z](rclcpp::Client<rviz_services::srv::MoveToPose>::SharedFuture response) {
            if (response.get()->success) {
                RCLCPP_INFO(logger_, "Move to pose successful");
                
                // Show success message in Qt thread
                QMetaObject::invokeMethod(this, [this]() {
                    status_label_->setText("Status: Move to pose successful");
                    status_label_->setStyleSheet("font-weight: bold; color: green;");
                    status_label_->setVisible(true);
                    
                    // Hide the status after 3 seconds
                    QTimer::singleShot(3000, this, [this]() {
                        status_label_->setVisible(false);
                    });
                }, Qt::QueuedConnection);
                
            } else {
                RCLCPP_ERROR(logger_, "Move to pose failed");
                
                // Show error message in Qt thread
                QMetaObject::invokeMethod(this, [this, x, y, z]() {
                    QMessageBox::warning(
                        this,
                        "Move to Pose Failed",
                        QString("Failed to move to position. The position may be unreachable.")
                    );
                }, Qt::QueuedConnection);
            }
        }
    );
}

// ============================
// Load File Tab Functions
// ============================

void rviz_teach_plugin::CustomPlugin::openFileDialog()
{
    // Open a file dialog to select a JSON file
    QString file_name = QFileDialog::getOpenFileName(this, "Select JSON File", "", "JSON Files (*.json)");
    if (file_name.isEmpty()) {
        // If no file is selected, return early
        return;
    }

    // Update the file path edit field with the selected file name
    file_path_edit_->setText(file_name);
    file_path_edit_->setCursorPosition(0);

    // Open the selected file for reading
    QFile file(file_name);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        // Log an error if the file cannot be opened
        RCLCPP_ERROR(logger_, "Failed to open file: %s", file_name.toStdString().c_str());
        QMessageBox::critical(this, "Error", "Failed to open file.");
        return;
    }

    // Read the file content into a QByteArray
    QByteArray data = file.readAll();
    file.close();

    // Parse the file content as a JSON document
    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (doc.isNull() || !doc.isObject()) {
        // Log an error if the file is not a valid JSON object
        RCLCPP_ERROR(logger_, "Invalid JSON file: %s", file_name.toStdString().c_str());
        return;
    }

    // Extract the root JSON object
    QJsonObject obj = doc.object();
    if (obj.contains("points") && obj["points"].isArray()) {
        // Clear the current point list and waypoint data
        point_list_->clear();
        waypoint_data_.clear();

        // Extract the array of points from the JSON object
        QJsonArray points = obj["points"].toArray();
        QString first_waypoint_name;

        // Iterate through each point in the array
        for (const auto &value : points) {
            QJsonObject point = value.toObject();
            if (point.contains("name") && point.contains("joint1")) {
                // Add the point name to the list and store its data
                QString name = point["name"].toString();
                point_list_->addItem(name);
                waypoint_data_[name] = point;
            }
        }

        // If at least one waypoint exists, load its details
        if (!first_waypoint_name.isEmpty()) {
            loadWaypointDetails(first_waypoint_name);
        }
    }
}

void rviz_teach_plugin::CustomPlugin::closeFile()
{
    file_path_edit_->clear();
    file_path_edit_->setPlaceholderText("Select JSON file...");
    point_list_->clear();
    waypoint_data_.clear();
    RCLCPP_INFO(logger_, "File closed and points cleared");

}

void rviz_teach_plugin::CustomPlugin::loadWaypointDetails(const QString &waypoint_name)
{
    // Check if the waypoint exists in the stored data
    if (waypoint_data_.find(waypoint_name) != waypoint_data_.end()) {
        // Retrieve the waypoint data
        QJsonObject point_data = waypoint_data_[waypoint_name];

        // Log the waypoint details
        RCLCPP_INFO(logger_, "Waypoint: %s", waypoint_name.toStdString().c_str());
        RCLCPP_INFO(logger_, "Joint1: %f, Joint2: %f, Joint3: %f, Joint4: %f, Gripper: %f",
                    point_data["joint1"].toDouble(), point_data["joint2"].toDouble(),
                    point_data["joint3"].toDouble(), point_data["joint4"].toDouble(),
                    point_data["gripper"].toDouble());
    }
}


// ============================
// Execution Functions
// ============================

void rviz_teach_plugin::CustomPlugin::runAllPoints()
{
    if (is_executing_points_) {
        RCLCPP_WARN(logger_, "Already executing!");
        return;
    }
    
    if (point_list_->count() == 0) {
        RCLCPP_WARN(logger_, "No waypoints available.");
        return;
    }
    
    // Prepare the list of waypoints to execute
    pending_waypoints_.clear();
    for (int i = 0; i < point_list_->count(); ++i) {
        pending_waypoints_ << point_list_->item(i)->text();
    }
    
    is_executing_points_ = true;
    RCLCPP_INFO(logger_, "Starting execution of all waypoints...");
    
    status_label_->setText("Status: Starting execution...");
    status_label_->setStyleSheet("font-weight: bold; color: orange;");
    status_label_->setVisible(true);
    cancel_run_button_->setVisible(true);
    
    executeNextWaypoint();
}

void rviz_teach_plugin::CustomPlugin::executeNextWaypoint()
{
    // Check if there are no more waypoints to execute
    if (pending_waypoints_.isEmpty()) {
        RCLCPP_INFO(logger_, "All waypoints have been executed.");
        is_executing_points_ = false;

        // Update the status label to indicate completion
        status_label_->setText("Status: All waypoints executed.");
        status_label_->setStyleSheet("font-weight: bold; color: green;");

        // Show a non-blocking message box to indicate completion
        QTimer::singleShot(0, this, [this]() {
        QMessageBox *box = new QMessageBox(QMessageBox::Information,
                                            "Completed",
                                            "All waypoints have been successfully executed.",
                                            QMessageBox::Ok,
                                            this);
        box->setAttribute(Qt::WA_DeleteOnClose);  // Automatically delete on close
        box->show();  // Do not use exec()!
        });

        status_label_->setVisible(false);
        cancel_run_button_->setVisible(false);

        return;
    }

    // Get the next waypoint from the pending list
    QString next_point = pending_waypoints_.takeFirst();

    // Update the status label to indicate the current waypoint being executed
    status_label_->setText("Status: Executing → " + next_point);
    status_label_->setStyleSheet("font-weight: bold; color: orange;");
    status_label_->setVisible(true);

    RCLCPP_INFO(logger_, "Sending waypoint: %s", next_point.toStdString().c_str());

    // Highlight the current waypoint in the list
    for (int i = 0; i < point_list_->count(); ++i) {
        if (point_list_->item(i)->text() == next_point) {
        point_list_->setCurrentRow(i);
        break;
        }
    }

    // Move the robot to the current waypoint
    moveToPoint();
}

void rviz_teach_plugin::CustomPlugin::cancelExecution()
{
     // Check if there is an active execution to cancel
    if (!is_executing_points_) {
        RCLCPP_INFO(logger_, "No active execution to cancel.");
        return;
    }

    RCLCPP_WARN(logger_, "Execution canceled by user.");

    // Reset the execution status
    is_executing_points_ = false;
    pending_waypoints_.clear();
    cancel_run_button_->setVisible(false);
    status_label_->setText("Status: Operation canceled.");
    status_label_->setStyleSheet("font-weight: bold; color: red;");

    // Optional feedback with a non-blocking message box
    QMessageBox *box = new QMessageBox(this);
    box->setWindowTitle("Canceled");
    box->setText("Execution has been canceled.");
    box->setIcon(QMessageBox::Warning);
    box->setStandardButtons(QMessageBox::Ok);
    box->show();
    status_label_->setVisible(false);

    // Call the stop service (if available)
    if (cancel_motion_client_ && cancel_motion_client_->wait_for_service(std::chrono::seconds(1))) {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = cancel_motion_client_->async_send_request(request);
        try {
        auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(logger_, "Motion successfully stopped: %s", result->message.c_str());
        } else {
            RCLCPP_WARN(logger_, "Stop service responded but was not successful.");
        }
        } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "Error calling cancel_motion: %s", e.what());
        }
    } else {
        RCLCPP_ERROR(logger_, "Stop service /cancel_motion not available.");
    }
}

void rviz_teach_plugin::CustomPlugin::moveToPoint()
{
    QListWidgetItem *selectedItem = point_list_->currentItem();
    if (!selectedItem) {
        RCLCPP_WARN(logger_, "No point selected!");
        return;
    }

    QString waypoint_name = selectedItem->text();
    if (waypoint_data_.find(waypoint_name) != waypoint_data_.end()) {
        QJsonObject point_data = waypoint_data_[waypoint_name];

        // Extract joint values
        double joint1 = point_data["joint1"].toDouble();
        double joint2 = point_data["joint2"].toDouble();
        double joint3 = point_data["joint3"].toDouble();
        double joint4 = point_data["joint4"].toDouble();
        double gripper = point_data["gripper"].toDouble();

        // Create and publish a JointState message
        sensor_msgs::msg::JointState msg;
        msg.name = {"joint2", "joint3", "joint1", "joint4", "joint_plate"};
        msg.position = {joint2, joint3, joint1, joint4, gripper};
        msg.header.stamp = node_->now();
        msg.header.frame_id = waypoint_name.toStdString();
        joint_pub_->publish(msg);

        RCLCPP_INFO(logger_, "Sent waypoint: %s", waypoint_name.toStdString().c_str());
        RCLCPP_INFO(logger_, "joint1: %.2f, joint2: %.2f, joint3: %.2f, joint4: %.2f, gripper: %.2f",
            joint1, joint2, joint3, joint4, gripper);
    } else {
        RCLCPP_WARN(logger_, "Waypoint %s not found in stored data!", waypoint_name.toStdString().c_str());
    }
}

// ============================
// Helper Functions
// ============================

sensor_msgs::msg::JointState::SharedPtr rviz_teach_plugin::CustomPlugin::fetchJointStates(const std::string &node_name, int timeout_seconds)
{
    // Create a temporary node to fetch joint states
    auto temp_node = rclcpp::Node::make_shared(node_name);

    // Promise to wait for the joint state message
    std::promise<sensor_msgs::msg::JointState::SharedPtr> msg_promise;
    auto future = msg_promise.get_future();

    // Create a subscription to the "/joint_states" topic
    auto subscription = temp_node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [&msg_promise](sensor_msgs::msg::JointState::SharedPtr msg) {
            msg_promise.set_value(msg);
        });

    // Use a single-threaded executor to spin the node
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(temp_node);

    // Wait for the message with a timeout
    if (executor.spin_until_future_complete(future, std::chrono::seconds(timeout_seconds)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(temp_node->get_logger(), "No /joint_states received within timeout!");
        return nullptr;
    }

    return future.get();
}

// Check if a waypoint with the given name already exists
bool rviz_teach_plugin::CustomPlugin::pointExists(const QString &name)
{
    return waypoint_data_.find(name) != waypoint_data_.end();
}

// Fetch and print joint states, and update sliders and labels
void rviz_teach_plugin::CustomPlugin::fetchAndPrintJointStates()
{
    // Rename this function to avoid confusion
    updateJointStateDisplay();
}

void rviz_teach_plugin::CustomPlugin::updateJointStateDisplay()
{
    if (!latest_joint_state_) return;

    // Don't update sliders if user is currently interacting with them
    for (const auto& pair : joint_slider_map_) {
        QSlider* slider = pair.second;
        if (slider && slider->isSliderDown()) {
            return; // User is actively using a slider, skip update
        }
    }

    for (size_t i = 0; i < latest_joint_state_->name.size(); ++i)
    {
        const std::string &joint_name = latest_joint_state_->name[i];
        double position_rad = latest_joint_state_->position[i];

        QString qt_name = QString::fromStdString(joint_name);

        double value = 0.0;
        QString value_str;

        if (qt_name == "joint_plate") {
            double mm = position_rad * 1000.0;
            int percent = static_cast<int>(std::round(mm / 10.0 * 100.0));
            percent = std::clamp(percent, 0, 100);
            value_str = QString::number(percent) + "%";
            value = percent;
        } else {
            value = position_rad * 180.0 / M_PI;
            value_str = QString::number(value, 'f', 1) + "°";
        }

        // Update the corresponding slider and label only if not being actively used
        if (joint_slider_map_.find(qt_name) != joint_slider_map_.end()) {
            QSlider* slider = joint_slider_map_[qt_name];
            QLabel* label = joint_label_map_[qt_name];

            int slider_val = static_cast<int>(std::round(value));

            if (slider && !slider->isSliderDown()) {
                slider->blockSignals(true);
                slider->setValue(slider_val);
                slider->blockSignals(false);
            }

            // Always update label if not actively being changed by user
            if (label && (!slider || !slider->isSliderDown())) {
                label->setText(value_str);
            }
        }
    }
}

void rviz_teach_plugin::CustomPlugin::fetchCurrentPose()
{
    try {
        // Look up the transform from base_link to link4_1 (end effector)
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform("base_link", "link4_1", tf2::TimePointZero);

        // Extract position from the transform
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;
        double z = transform.transform.translation.z;

        // Update the text fields with the position data (using 3 decimal places)
        x_coord_edit_->setText(QString::number(x, 'f', 3));
        y_coord_edit_->setText(QString::number(y, 'f', 3));
        z_coord_edit_->setText(QString::number(z, 'f', 3));

        RCLCPP_INFO(logger_, "Current end effector position: X=%.3f, Y=%.3f, Z=%.3f", x, y, z);
        
        // Show a brief status message
        status_label_->setText("Status: Current position fetched");
        status_label_->setStyleSheet("font-weight: bold; color: green;");
        status_label_->setVisible(true);
        
        // Hide the status after 2 seconds
        QTimer::singleShot(2000, this, [this]() {
            status_label_->setVisible(false);
        });
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(logger_, "Could not transform from base_link to link4_1: %s", ex.what());
        
        // Show an error message
        QMessageBox::warning(
            this,
            "Transform Unavailable",
            QString("Failed to get the current position: %1").arg(ex.what())
        );
    }
}
// ============================
// Plugin Export
// ============================

#include "teach_plugin.moc"
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_teach_plugin::CustomPlugin, rviz_common::Panel)