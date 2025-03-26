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
#include <rclcpp/rclcpp.hpp>
#include <QDebug>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <random>
#include <QComboBox>
#include "std_srvs/srv/set_bool.hpp"
#include <thread>
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <QTimer>
#include <QMessageBox>
#include "rviz_services/srv/move_linear.hpp"


namespace rviz_teach_plugin
{

class CustomPlugin : public rviz_common::Panel
{
Q_OBJECT

public:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_ = rclcpp::get_logger("CustomPlugin");
    QListWidget* point_list_;
    QLineEdit* file_path_edit_;
    QLabel* status_label_;
    QPushButton* cancel_run_button_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_motion_client_;
    std::map<QString, QJsonObject> waypoint_data_;
    QStringList pending_waypoints_;
    bool is_executing_points_ = false;
    double movement_step_size_ =10.0;

    // Service clients for X, Z, and Home movements
    rclcpp::Client<rviz_services::srv::MoveLinear>::SharedPtr move_linear_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr home_client_;

  CustomPlugin(QWidget *parent = nullptr)
  : rviz_common::Panel(parent)
  {
    node_ = std::make_shared<rclcpp::Node>("rviz_teach_plugin");
    pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("move_robot", 10);
    joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("target_joint_states", 10);
    cancel_motion_client_ = node_->create_client<std_srvs::srv::Trigger>("cancel_motion");

    // Create service clients
    move_linear_client_ = node_->create_client<rviz_services::srv::MoveLinear>("move_linear");
    home_client_ = node_->create_client<std_srvs::srv::SetBool>("move_to_home");

    auto *tab_widget = new QTabWidget;
    auto *main_layout = new QVBoxLayout; 
    
    // Teach Tab
    QWidget *teach_tab = new QWidget;
    QVBoxLayout *teach_layout = new QVBoxLayout;
    point_list_ = new QListWidget;
    teach_layout->addWidget(point_list_);
    auto *edit_button = new QPushButton("Edit Point Name");
    teach_layout->addWidget(edit_button);

    auto *save_delete_layout = new QHBoxLayout;
    auto *save_button = new QPushButton("Save JSON");
    auto *delete_point_button = new QPushButton("Delete Point");
    save_delete_layout->addWidget(save_button);
    save_delete_layout->addWidget(delete_point_button);
    teach_layout->addLayout(save_delete_layout);

    auto *move_buttons_layout = new QHBoxLayout;
    auto *move_point_up_button = new QPushButton("Move Point Up");
    auto *move_point_down_button = new QPushButton("Move Point Down");
    move_buttons_layout->addWidget(move_point_up_button);
    move_buttons_layout->addWidget(move_point_down_button);
    teach_layout->addLayout(move_buttons_layout);

    auto *teach_button = new QPushButton("Teach Point");
    teach_layout->addWidget(teach_button);

    auto *move_run_buttons_layout = new QVBoxLayout;
    auto *button_row = new QHBoxLayout;
    auto *move_to_point_button = new QPushButton("Move to Point");
    auto *run_all_points_button = new QPushButton("Run all Points");
    button_row->addWidget(move_to_point_button);
    button_row->addWidget(run_all_points_button);
    move_run_buttons_layout->addLayout(button_row);
    
    
    // Status-Label vorbereiten, aber leer lassen
    status_label_ = new QLabel();
    status_label_->setVisible(false);  // Anfangs nicht sichtbar
    status_label_->setStyleSheet("font-weight: bold; color: orange;");
    move_run_buttons_layout->addWidget(status_label_);

    cancel_run_button_ = new QPushButton("Abbrechen");
    cancel_run_button_->setVisible(false);  // Anfangs ausgeblendet
    button_row->addWidget(cancel_run_button_);
        
    teach_layout->addLayout(move_run_buttons_layout);
/*
    auto *move_run_buttons_layout = new QHBoxLayout;
    auto *move_to_point_button = new QPushButton("Move to Point");
    auto *run_all_points_button = new QPushButton("Run all Points");
    move_run_buttons_layout->addWidget(move_to_point_button);
    move_run_buttons_layout->addWidget(run_all_points_button);
    teach_layout->addLayout(move_run_buttons_layout);
*/
    teach_tab->setLayout(teach_layout);
    tab_widget->addTab(teach_tab, "Teach");
 
    // Move Robot Tab
    QWidget *move_tab = new QWidget;
    QVBoxLayout *move_layout = new QVBoxLayout;

    // Sliders for Joint Angles and Gripper
    auto *joint_control_label = new QLabel("Joint Controls:");
    move_layout->addWidget(joint_control_label);
    for (int i = 1; i <= 4; ++i) {
        auto *joint_layout = new QHBoxLayout;
        auto *joint_label = new QLabel(QString("Joint %1 Angle:").arg(i));
        auto *joint_value = new QLabel("0°");
        joint_value->setFixedWidth(50);
        auto *joint_slider = new QSlider(Qt::Horizontal);
        joint_slider->setRange(-180, 180);
        joint_layout->addWidget(joint_label);
        joint_layout->addWidget(joint_slider);
        joint_layout->addWidget(joint_value);
        move_layout->addLayout(joint_layout);

        connect(joint_slider, &QSlider::valueChanged, [joint_value](int value){
            joint_value->setText(QString::number(value) + "°");
        });
        move_layout->addWidget(joint_slider);
    }
    auto *gripper_label = new QLabel("Gripper Opening:");
    auto *gripper_slider = new QSlider(Qt::Horizontal);
    gripper_slider->setRange(0, 100);
    auto *gripper_layout = new QHBoxLayout;
    gripper_layout->addWidget(gripper_label);
    auto *gripper_value = new QLabel("0%");
    gripper_value->setFixedWidth(50);
    gripper_layout->addWidget(gripper_slider);
    gripper_layout->addWidget(gripper_value);
    move_layout->addLayout(gripper_layout);

    connect(gripper_slider, &QSlider::valueChanged, [gripper_value](int value){
        gripper_value->setText(QString::number(value) + "%");
    });
    
    // Section for controlling linear movements
    auto *lin_movement_label = new QLabel("Linear Movements:");
    move_layout->addWidget(lin_movement_label);
    
    // Dropdown for step size (1mm or 10mm)
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
    lin_layout->addWidget(move_x_button);
    lin_layout->addWidget(move_x_neg_button);
    lin_layout->addWidget(move_z_button);
    lin_layout->addWidget(move_z_neg_button);
    lin_layout->addWidget(home_button);
    move_layout->addLayout(lin_layout);
    
    // Add Waypoint Button
    auto *add_point_button = new QPushButton("Add Point");
    move_layout->addWidget(add_point_button);
    
    move_tab->setLayout(move_layout);
    tab_widget->addTab(move_tab, "Move Robot");

   // Load File Tab
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
    
    // Connect buttons
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
    connect(teach_button, &QPushButton::clicked, this, &CustomPlugin::onTeachPoint);
    connect(step_size_dropdown, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CustomPlugin::setStepSize);
    connect(move_x_button, &QPushButton::clicked, this, [this]() { sendMovementRequest(true, true); });
    connect(move_x_neg_button, &QPushButton::clicked, this, [this]() { sendMovementRequest(true, false); });
    connect(move_z_button, &QPushButton::clicked, this, [this]() { sendMovementRequest(false, true); });
    connect(move_z_neg_button, &QPushButton::clicked, this, [this]() { sendMovementRequest(false, false); });
    connect(home_button, &QPushButton::clicked, this, &CustomPlugin::onMoveToHome);

/*
    status_label_ = new QLabel();
    status_label_->setVisible(false);
    status_label_->setStyleSheet("font-weight: bold; color: orange;");
 */   
    // Start the ROS2 node in a separate thread
    ros_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    // Feedback vom Roboter abonnieren
    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "robot_execution_status", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(logger_, "Feedback vom Roboter: %s", msg->data.c_str());
     
        if (is_executing_points_) {
          executeNextWaypoint();
        }
      }
    );

  }

  ~CustomPlugin()
  {
    rclcpp::shutdown();
    if (ros_thread_.joinable()) {
      ros_thread_.join();
    }
  }

private Q_SLOTS:
  void setStepSize(int index)
  {
    QComboBox *step_size_dropdown = qobject_cast<QComboBox*>(sender());
    if (step_size_dropdown) {
        movement_step_size_ = step_size_dropdown->itemData(index).toDouble();
        RCLCPP_INFO(logger_, "Step size changed to %f mm", movement_step_size_);
    }
  }

  void sendMovementRequest(bool is_x_direction, bool is_positive) {
      if (!move_linear_client_->wait_for_service(std::chrono::seconds(2))) {
          RCLCPP_ERROR(logger_, "Service move_linear not available");
          return;
      }
  
      auto request = std::make_shared<rviz_services::srv::MoveLinear::Request>();
      request->direction = is_x_direction ? "X" : "Z";
      request->distance_mm = movement_step_size_ * (is_positive ? 1.0 : -1.0);  // Use distance_mm instead of step_size
  
      auto future = move_linear_client_->async_send_request(request);
      try {
          auto response = future.get();
          if (response->success) {
              RCLCPP_INFO(logger_, "Move %s successful", request->direction.c_str());
          } else {
              RCLCPP_ERROR(logger_, "Move %s failed", request->direction.c_str());
          }
      } catch (const std::exception &e) {
          RCLCPP_ERROR(logger_, "Service call failed: %s", e.what());
      }
    }


  void onMoveToHome()
  {
    if (!home_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(logger_, "Service /move_to_home not available");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto future = home_client_->async_send_request(request);
    try {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(logger_, "Move to home successful: %s", response->message.c_str());
      } else {
        RCLCPP_ERROR(logger_, "Move to home failed: %s", response->message.c_str());
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger_, "Service call failed: %s", e.what());
    }
  }

  void onTeachPoint()
  {
    QListWidgetItem *selectedItem = point_list_->currentItem();
    if (!selectedItem) {
        RCLCPP_WARN(logger_, "No point selected!");
        return;
    }

    QString waypoint_name = selectedItem->text();
    if (waypoint_data_.find(waypoint_name) != waypoint_data_.end()) {
         // Temporärer Node zum einmaligen Abrufen
        auto temp_node = rclcpp::Node::make_shared("joint_state_reader_temp");
        auto sub = temp_node->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", 10,
          [](const sensor_msgs::msg::JointState::SharedPtr) { /* Nur Dummy */ });

        // Warten auf eine Nachricht mit Timeout
        sensor_msgs::msg::JointState::SharedPtr msg = nullptr;
        auto start = std::chrono::steady_clock::now();
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription;

        std::promise<sensor_msgs::msg::JointState::SharedPtr> msg_promise;
        auto future = msg_promise.get_future();

        subscription = temp_node->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", 10,
          [&msg_promise](sensor_msgs::msg::JointState::SharedPtr m) {
            msg_promise.set_value(m);
          });

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(temp_node);

        if (executor.spin_until_future_complete(future, std::chrono::seconds(2)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_WARN(temp_node->get_logger(), "Kein /joint_states erhalten!");
          return;
        }

        msg = future.get();

        // Mapping: Name → Position
        std::map<std::string, double> joint_map;
        for (size_t i = 0; i < msg->name.size(); ++i) {
          joint_map[msg->name[i]] = msg->position[i];
        }

        // Werte extrahieren
        double joint1 = joint_map["joint1"];
        double joint2 = joint_map["joint2"];
        double joint3 = joint_map["joint3"];
        double joint4 = joint_map["joint4"];
        double gripper = joint_map["joint_plate"];  // Mapping!

      
        waypoint_data_[waypoint_name]["joint1"] = joint1;
        waypoint_data_[waypoint_name]["joint2"] = joint2;
        waypoint_data_[waypoint_name]["joint3"] = joint3;
        waypoint_data_[waypoint_name]["joint4"] = joint4;
        waypoint_data_[waypoint_name]["gripper"] = gripper;
    }

    RCLCPP_INFO(logger_, "Updated values for %s", waypoint_name.toStdString().c_str());
}

  void addWaypoint()
  {
    QString base_name = "Waypoint";
    int index = 1;
    QString waypoint_name;
    do {
        waypoint_name = base_name + "_" + QString::number(index);
        index++;
    } while (pointExists(waypoint_name));

      // Temporärer Node zum einmaligen Abrufen
    auto temp_node = rclcpp::Node::make_shared("joint_state_reader_temp");
    auto sub = temp_node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [](const sensor_msgs::msg::JointState::SharedPtr) { /* Nur Dummy */ });

    // Warten auf eine Nachricht mit Timeout
    sensor_msgs::msg::JointState::SharedPtr msg = nullptr;
    auto start = std::chrono::steady_clock::now();
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription;

    std::promise<sensor_msgs::msg::JointState::SharedPtr> msg_promise;
    auto future = msg_promise.get_future();

    subscription = temp_node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [&msg_promise](sensor_msgs::msg::JointState::SharedPtr m) {
        msg_promise.set_value(m);
      });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(temp_node);

    if (executor.spin_until_future_complete(future, std::chrono::seconds(2)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(temp_node->get_logger(), "Kein /joint_states erhalten!");
      return;
    }

    msg = future.get();

    // Mapping: Name → Position
    std::map<std::string, double> joint_map;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_map[msg->name[i]] = msg->position[i];
    }

    // Werte extrahieren
    double joint1 = joint_map["joint1"];
    double joint2 = joint_map["joint2"];
    double joint3 = joint_map["joint3"];
    double joint4 = joint_map["joint4"];
    double gripper = joint_map["joint_plate"];  // Mapping!

   
    // Speichern
    QJsonObject new_point;
    new_point["name"] = waypoint_name;
    new_point["joint1"] = joint1;
    new_point["joint2"] = joint2;
    new_point["joint3"] = joint3;
    new_point["joint4"] = joint4;
    new_point["gripper"] = gripper;

    // Punkt zur Liste hinzufügen
    point_list_->addItem(waypoint_name);
    waypoint_data_[waypoint_name] = new_point;

    RCLCPP_INFO(logger_, "Neuer Punkt gespeichert: %s", waypoint_name.toStdString().c_str());

  }

  bool pointExists(const QString &name)
  {
    return waypoint_data_.find(name) != waypoint_data_.end();
    }


    void saveWaypointToJson(const QString &name, const QStringList &joint_values)
    {
        QString file_name = "waypoints.json";
        QFile file(file_name);
        QJsonArray points;

        if (file.exists() && file.open(QIODevice::ReadOnly)) {
            QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
            file.close();
            if (doc.isObject() && doc.object().contains("points")) {
                points = doc.object()["points"].toArray();
            }
        }

        QJsonObject new_point;
        new_point["name"] = name;
        new_point["joint1"] = joint_values[0].toDouble();
        new_point["joint2"] = joint_values[1].toDouble();
        new_point["joint3"] = joint_values[2].toDouble();
        new_point["joint4"] = joint_values[3].toDouble();
        new_point["gripper"] = joint_values[4].toDouble();
        points.append(new_point);

        QJsonObject root;
        root["points"] = points;
        file.open(QIODevice::WriteOnly);
        file.write(QJsonDocument(root).toJson());
        file.close();

        RCLCPP_INFO(rclcpp::get_logger("CustomPlugin"), "Neuer Wegpunkt '%s' gespeichert!", name.toStdString().c_str());
    }


    void moveToPoint()
    {
      QListWidgetItem *selectedItem = point_list_->currentItem();
      if (!selectedItem) {
          RCLCPP_WARN(logger_, "No point selected!");
          return;
      }
    
      QString waypoint_name = selectedItem->text();
      if (waypoint_data_.find(waypoint_name) != waypoint_data_.end()) {
          QJsonObject point_data = waypoint_data_[waypoint_name];
    
          double joint1 = point_data["joint1"].toDouble();
          double joint2 = point_data["joint2"].toDouble();
          double joint3 = point_data["joint3"].toDouble();
          double joint4 = point_data["joint4"].toDouble();
          double gripper = point_data["gripper"].toDouble();
    
          sensor_msgs::msg::JointState msg;
          // ✨ Diese Reihenfolge erwartet dein joint_listener
          msg.name = {"joint2", "joint3", "joint1", "joint4", "joint_plate"};
          msg.position = {joint2, joint3, joint1, joint4, gripper};
          msg.header.stamp = node_->now();
          msg.header.frame_id = waypoint_name.toStdString();
          joint_pub_->publish(msg);
    
          RCLCPP_INFO(logger_, "Gesendeter Waypoint: %s", waypoint_name.toStdString().c_str());
          RCLCPP_INFO(logger_, "joint1: %.2f, joint2: %.2f, joint3: %.2f, joint4: %.2f, gripper: %.2f",
                      joint1, joint2, joint3, joint4, gripper);
      } else {
          RCLCPP_WARN(logger_, "Waypoint %s not found in stored data!", waypoint_name.toStdString().c_str());
      }
    }
    

  void runAllPoints() 
  {
    
    if (is_executing_points_) {
      RCLCPP_WARN(logger_, "Bereits in Ausführung!");
      return;
    }
  
    if (point_list_->count() == 0) {
      RCLCPP_WARN(logger_, "Keine Wegpunkte vorhanden.");
      return;
    }
  
    pending_waypoints_.clear();
    for (int i = 0; i < point_list_->count(); ++i) {
      pending_waypoints_ << point_list_->item(i)->text();
    }
  
    is_executing_points_ = true;
    RCLCPP_INFO(logger_, "Starte Abarbeitung aller Punkte...");

    status_label_->setText("Status: Starte Abarbeitung...");
    status_label_->setStyleSheet("font-weight: bold; color: orange;");
  
    status_label_->setVisible(true);
    cancel_run_button_->setVisible(true);
    status_label_->setText("Status: Starte Abarbeitung...");
    status_label_->setStyleSheet("font-weight: bold; color: orange;");
    executeNextWaypoint();
  }

  void closeFile() 
  {
    file_path_edit_->clear();
    file_path_edit_->setPlaceholderText("Select JSON file...");
    point_list_->clear();
    waypoint_data_.clear();
    RCLCPP_INFO(logger_, "File closed and points cleared");
}

void cancelExecution()
{
  if (!is_executing_points_) {
    RCLCPP_INFO(logger_, "Kein aktiver Ablauf zum Abbrechen.");
    return;
  }

  RCLCPP_WARN(logger_, "Abarbeitung durch Benutzer abgebrochen.");
  is_executing_points_ = false;
  pending_waypoints_.clear();

  cancel_run_button_->setVisible(false);

  if (cancel_motion_client_->wait_for_service(std::chrono::seconds(1))) {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = cancel_motion_client_->async_send_request(request);
    try {
      auto result = future.get();
      if (result->success) {
        RCLCPP_INFO(logger_, "Bewegung wurde erfolgreich gestoppt: %s", result->message.c_str());
      } else {
        RCLCPP_WARN(logger_, "Stop-Service antwortete, aber nicht erfolgreich.");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger_, "Fehler beim Aufruf von cancel_motion: %s", e.what());
    }
  } else {
    RCLCPP_ERROR(logger_, "Stop-Service /cancel_motion nicht verfügbar.");
  }

  status_label_->setText("Status: Vorgang abgebrochen.");
  status_label_->setStyleSheet("font-weight: bold; color: red;");
  status_label_->setVisible(false);

  QMessageBox::warning(this, "Abgebrochen", "Die Ausführung wurde abgebrochen.");
}

void executeNextWaypoint()
{
  if (pending_waypoints_.isEmpty()) {
    RCLCPP_INFO(logger_, "Alle Punkte wurden ausgeführt.");
    is_executing_points_ = false;

    status_label_->setText("Status: Alle Punkte ausgeführt.");
    status_label_->setStyleSheet("font-weight: bold; color: green;");
    status_label_->setVisible(true);
    cancel_run_button_->setVisible(false);

    QMessageBox::information(this, "Fertig", "Alle Punkte wurden erfolgreich ausgeführt.");
    return;
  }

  status_label_->setText("Status: Ausführen → " + pending_waypoints_.first());
  status_label_->setStyleSheet("font-weight: bold; color: orange;");
  status_label_->setVisible(true);

  QString next_point = pending_waypoints_.takeFirst();
  RCLCPP_INFO(logger_, "Sende Punkt: %s", next_point.toStdString().c_str());

  for (int i = 0; i < point_list_->count(); ++i) {
    if (point_list_->item(i)->text() == next_point) {
      point_list_->setCurrentRow(i);
      break;
    }
  }

  moveToPoint();
}
 

void openFileDialog() 
{
  QString file_name = QFileDialog::getOpenFileName(this, "Select JSON File", "", "JSON Files (*.json)");
  if (file_name.isEmpty()) {
      return;
  }

  file_path_edit_->setText(file_name);
  file_path_edit_->setCursorPosition(0);

  QFile file(file_name);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      RCLCPP_ERROR(logger_, "Failed to open file: %s", file_name.toStdString().c_str());
      return;
  }

  QByteArray data = file.readAll();
  file.close();
  QJsonDocument doc = QJsonDocument::fromJson(data);
  if (doc.isNull() || !doc.isObject()) {
      RCLCPP_ERROR(logger_, "Invalid JSON file: %s", file_name.toStdString().c_str());
      return;
  }

  QJsonObject obj = doc.object();
  if (obj.contains("points") && obj["points"].isArray()) {
      point_list_->clear();
      waypoint_data_.clear();
      QJsonArray points = obj["points"].toArray();
      QString first_waypoint_name;

      for (const auto &value : points) {
          QJsonObject point = value.toObject();
          if (point.contains("name") && point.contains("joint1")) {
              QString name = point["name"].toString();
              point_list_->addItem(name);
              waypoint_data_[name] = point;
          
          }
      }

      // Falls mindestens ein Wegpunkt existiert, lade automatisch dessen Details
      if (!first_waypoint_name.isEmpty()) {
          loadWaypointDetails(first_waypoint_name);
      }
  }
}

  void loadWaypointDetails(const QString &waypoint_name)
    {
        if (waypoint_data_.find(waypoint_name) != waypoint_data_.end()) {
            QJsonObject point_data = waypoint_data_[waypoint_name];
            RCLCPP_INFO(logger_, "Waypoint: %s", waypoint_name.toStdString().c_str());
            RCLCPP_INFO(logger_, "Joint1: %f, Joint2: %f, Joint3: %f, Joint4: %f, Gripper: %f",
                        point_data["joint1"].toDouble(), point_data["joint2"].toDouble(),
                        point_data["joint3"].toDouble(), point_data["joint4"].toDouble(),
                        point_data["gripper"].toDouble());
        }
    }
  
    void saveJsonFile()
    {
        if (point_list_->count() == 0) {
            RCLCPP_WARN(logger_, "No points available to save.");
            return;
        }
        QString file_name = QFileDialog::getSaveFileName(this, "Save JSON File", "", "JSON Files (*.json)");
        if (file_name.isEmpty()) {
            return;
        }
    
        QFile file(file_name);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            RCLCPP_ERROR(logger_, "Failed to save file: %s", file_name.toStdString().c_str());
            return;
        }
    
        QJsonArray points;
        for (int i = 0; i < point_list_->count(); ++i) {
            QString name = point_list_->item(i)->text();
            if (waypoint_data_.find(name) != waypoint_data_.end()) {
                points.append(waypoint_data_[name]);
            }
        }
    
        QJsonObject obj;
        obj["points"] = points;
        QJsonDocument doc(obj);
        file.write(doc.toJson());
        file.close();
    }
     
  void editPoint()
  {
    QListWidgetItem *selectedItem = point_list_->currentItem();
    if (selectedItem)
    {
        QString oldName = selectedItem->text();
        bool ok;
        QString newName = QInputDialog::getText(nullptr, "Edit Point", "Enter new point name:", QLineEdit::Normal, oldName, &ok);
        
        if (ok && !newName.isEmpty() && newName != oldName && !pointExists(newName))
        {
            // Umbenennen des Punktes in der Liste
            selectedItem->setText(newName);

            // Umbenennen der zugehörigen Daten
            auto it = waypoint_data_.find(oldName);
            if (it != waypoint_data_.end()) {
                QJsonObject pointData = it->second;
                pointData["name"] = newName;  
                waypoint_data_.erase(it);  
                waypoint_data_[newName] = pointData;  
            }
        }
    }
}


void deletePoint()
{
  QListWidgetItem *selectedItem = point_list_->currentItem();
  if (selectedItem) {
      QString waypoint_name = selectedItem->text();
      waypoint_data_.erase(waypoint_name);
      delete point_list_->takeItem(point_list_->row(selectedItem));
  }
}

  void movePointUp()
  {
    int currentIndex = point_list_->currentRow();
    if (currentIndex > 0) {
        QListWidgetItem *item = point_list_->takeItem(currentIndex);
        point_list_->insertItem(currentIndex - 1, item);
        point_list_->setCurrentItem(item);
    }
  }

  void movePointDown()
  {
    int currentIndex = point_list_->currentRow();
    if (currentIndex >= 0 && currentIndex < point_list_->count() - 1) {
        QListWidgetItem *item = point_list_->takeItem(currentIndex);
        point_list_->insertItem(currentIndex + 1, item);
        point_list_->setCurrentItem(item);
    }
  }

  private:
  std::thread ros_thread_;  // Declare the ROS thread here
};

}  // namespace rviz_teach_plugin

#include "teach_plugin.moc"
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_teach_plugin::CustomPlugin, rviz_common::Panel)