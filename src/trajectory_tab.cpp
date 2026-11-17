// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include "surface_gui/trajectory_tab.hpp"

#include <map>
#include <string>
#include <vector>

#include <QCheckBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QtMath>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>

#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <surface_raster_planner_msgs/srv/load_mesh.hpp>
#include <scene_tools_interfaces/srv/add_object.hpp>
#include <time_parametrization_msgs/action/apply_time_parametrization.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace surface_gui
{

namespace
{
constexpr int kLoadTimeoutMs = 60000;
constexpr int kRasterTimeoutMs = 60000;

QDoubleSpinBox * makeSpinBox(
  double min, double max, double step, int decimals, double value, const QString & suffix)
{
  auto spin = new QDoubleSpinBox;
  spin->setRange(min, max);
  spin->setSingleStep(step);
  spin->setDecimals(decimals);
  spin->setValue(value);
  spin->setSuffix(suffix);
  return spin;
}

double seconds(const builtin_interfaces::msg::Duration & d)
{
  return d.sec + d.nanosec * 1e-9;
}
}  // namespace

TrajectoryTab::TrajectoryTab(QWidget * parent)
: TabBase(parent)
{
  // --- 1. Mesh
  load_topic_button_ = new QPushButton("Load from topic");
  load_topic_button_->setToolTip("Use the last mesh published by the reconstruction node");
  mesh_file_ = new QLineEdit;
  mesh_file_->setPlaceholderText("mesh file (.ply, .obj, .vtk, .stl)");
  browse_button_ = new QPushButton("...");
  browse_button_->setMaximumWidth(32);
  mesh_frame_ = new QLineEdit;
  mesh_frame_->setPlaceholderText("empty: node parameter mesh_frame");
  load_file_button_ = new QPushButton("Load from file");
  mesh_label_ = new QLabel("No mesh loaded");
  mesh_label_->setWordWrap(true);

  auto mesh_box = new QGroupBox("1. Mesh");
  auto mesh_layout = new QVBoxLayout(mesh_box);
  mesh_layout->addWidget(load_topic_button_);
  auto file_row = new QHBoxLayout;
  file_row->addWidget(mesh_file_);
  file_row->addWidget(browse_button_);
  mesh_layout->addLayout(file_row);
  auto frame_row = new QFormLayout;
  frame_row->addRow("Frame", mesh_frame_);
  mesh_layout->addLayout(frame_row);
  mesh_layout->addWidget(load_file_button_);
  mesh_layout->addWidget(mesh_label_);

  // --- 2. Collision object
  object_file_ = new QLineEdit;
  object_file_->setPlaceholderText("object file (.ply, .obj, .vtk, .stl)");
  browse_obj_button_ = new QPushButton("...");
  browse_obj_button_->setMaximumWidth(32);
  object_frame_ = new QLineEdit("table");
  object_frame_->setPlaceholderText("Collision object reference frame");
  load_obj_button_ = new QPushButton("Load from file");
  object_label_ = new QLabel("No object loaded");
  object_label_->setWordWrap(true);
  padding_ = makeSpinBox(0.0005, 1.0, 0.001, 4, 0.02, " m");

  auto collision_box = new QGroupBox("2. Collision Object");
  auto object_layout = new QVBoxLayout(collision_box);
  auto file_obj_row = new QHBoxLayout;
  file_obj_row->addWidget(object_file_);
  file_obj_row->addWidget(browse_obj_button_);
  object_layout->addLayout(file_obj_row);
  auto frame_obj_row = new QFormLayout;
  frame_obj_row->addRow("Frame", object_frame_);
  object_layout->addLayout(frame_obj_row);
  object_layout->addWidget(load_obj_button_);
  object_layout->addWidget(object_label_);
  auto padding_form = new QFormLayout;
  padding_form->addRow("Padding", padding_);
  object_layout->addLayout(padding_form);
  

  // --- 3. Raster path
  row_step_ = makeSpinBox(0.0005, 1.0, 0.001, 4, 0.02, " m");
  row_step_->setToolTip("Distance between two consecutive rows, measured across the rows");
  row_angle_ = makeSpinBox(-180.0, 180.0, 5.0, 1, 0.0, " deg");
  row_angle_->setToolTip(
    "Inclination of the rows with respect to the long side of the surface,\n"
    "counter-clockwise around the mean normal. 0 = along the long side, 90 = along the short side.");
  edge_margin_ = makeSpinBox(0.0, 1.0, 0.001, 4, 0.0, " m");
  edge_margin_->setToolTip(
    "Poses closer than this to the boundary of the surface (outline and holes) are dropped.\n"
    "0 disables.");
  keep_tool_orientation_ = new QCheckBox("Keep tool orientation");
  keep_tool_orientation_->setToolTip(
    "Checked: the tool x axis always follows the row direction, also on the return rows.\n"
    "Unchecked: the tool x axis follows the travel direction (flips on the return rows).");
  raster_button_ = new QPushButton("Compute raster path");
  raster_label_ = new QLabel;
  raster_label_->setWordWrap(true);

  auto raster_box = new QGroupBox("3. Raster path");
  auto raster_layout = new QVBoxLayout(raster_box);
  auto raster_form = new QFormLayout;
  raster_form->addRow("Row step", row_step_);
  raster_form->addRow("Row angle", row_angle_);
  raster_form->addRow("Edge margin", edge_margin_);
  raster_layout->addLayout(raster_form);
  raster_layout->addWidget(keep_tool_orientation_);
  raster_layout->addWidget(raster_button_);
  raster_layout->addWidget(raster_label_);

  // --- 4. Joint path
  planning_frame_ = new QLineEdit("open_tip");
  planning_frame_->setToolTip("Robot frame (TCP) that must reach the poses");
  auto deviation_row = new QHBoxLayout;
  const double default_deviation[3] = {0.0, 0.0, 0.05};
  const char * axes[3] = {"x ", "y ", "z "};
  for (int i = 0; i < 3; ++i) {
    deviation_[i] = makeSpinBox(-1.0, 1.0, 0.005, 3, default_deviation[i], " m");
    deviation_[i]->setPrefix(axes[i]);
    deviation_row->addWidget(deviation_[i]);
  }
  joint_path_button_ = new QPushButton("Compute joint path");
  joint_path_label_ = new QLabel;
  joint_path_label_->setWordWrap(true);

  auto joint_path_box = new QGroupBox("4. Joint path");
  auto joint_path_layout = new QVBoxLayout(joint_path_box);
  auto joint_path_form = new QFormLayout;
  joint_path_form->addRow("Planning frame", planning_frame_);
  joint_path_form->addRow("Leave/approach offset (tool frame)", deviation_row);
  joint_path_layout->addLayout(joint_path_form);
  joint_path_layout->addWidget(joint_path_button_);
  joint_path_layout->addWidget(joint_path_label_);

  // --- 5. Trajectory
  group_name_ = new QLineEdit("ur_robot");
  group_name_->setToolTip("MoveIt planning group");
  scaling_factor_ = makeSpinBox(0.01, 1.0, 0.05, 2, 0.1, "");
  scaling_factor_->setToolTip("Velocity and acceleration scaling factor");
  trajectory_button_ = new QPushButton("Compute trajectory");
  trajectory_button_->setToolTip(
    "Time parametrization of the joint path, starting from the current robot configuration");
  execute_button_ = new QPushButton("Execute");
  execute_button_->setToolTip("Execute the trajectory with MoveIt: the robot moves");
  trajectory_label_ = new QLabel;
  trajectory_label_->setWordWrap(true);

  auto trajectory_box = new QGroupBox("5. Trajectory");
  auto trajectory_layout = new QVBoxLayout(trajectory_box);
  auto trajectory_form = new QFormLayout;
  trajectory_form->addRow("Planning group", group_name_);
  trajectory_form->addRow("Scaling factor", scaling_factor_);
  trajectory_layout->addLayout(trajectory_form);
  auto trajectory_buttons = new QHBoxLayout;
  trajectory_buttons->addWidget(trajectory_button_);
  trajectory_buttons->addWidget(execute_button_);
  trajectory_layout->addLayout(trajectory_buttons);
  trajectory_layout->addWidget(trajectory_label_);

  // --- Progress
  progress_ = new QProgressBar;
  progress_->setTextVisible(true);
  progress_->setVisible(false);
  cancel_button_ = new QPushButton("Cancel");
  cancel_button_->setToolTip("Cancel the running action (it also stops the execution)");
  auto progress_row = new QHBoxLayout;
  progress_row->addWidget(progress_, 1);
  progress_row->addWidget(cancel_button_);

  // --- Interfaces
  raster_node_ = new QLineEdit("/raster_planner_node");
  joint_path_action_ = new QLineEdit("/compute_joint_path");
  time_parametrization_action_ = new QLineEdit("/compute_joint_trajectory");
  execute_action_ = new QLineEdit("/execute_trajectory");
  joint_states_topic_ = new QLineEdit("/joint_states");
  auto interfaces = new QWidget;
  auto interfaces_layout = new QFormLayout(interfaces);
  interfaces_layout->addRow("Raster planner node", raster_node_);
  interfaces_layout->addRow("Joint path action", joint_path_action_);
  interfaces_layout->addRow("Time param. action", time_parametrization_action_);
  interfaces_layout->addRow("Execute action", execute_action_);
  interfaces_layout->addRow("Joint states topic", joint_states_topic_);

  auto layout = new QVBoxLayout(this);
  layout->addWidget(mesh_box);
  layout->addWidget(collision_box);
  layout->addWidget(raster_box);
  layout->addWidget(joint_path_box);
  layout->addWidget(trajectory_box);
  layout->addLayout(progress_row);
  layout->addWidget(makeCollapsible("Interfaces", interfaces));
  layout->addStretch();

  connect(load_topic_button_, &QPushButton::clicked, this, &TrajectoryTab::loadMeshFromTopic);
  connect(load_file_button_, &QPushButton::clicked, this, &TrajectoryTab::loadMeshFromFile);
  connect(browse_button_, &QPushButton::clicked, this, &TrajectoryTab::browseMeshFile);

  connect(load_obj_button_, &QPushButton::clicked, this, &TrajectoryTab::loadObjectFromFile);
  connect(browse_obj_button_, &QPushButton::clicked, this, &TrajectoryTab::browseObjectFile);

  connect(raster_button_, &QPushButton::clicked, this, &TrajectoryTab::computeRaster);
  connect(joint_path_button_, &QPushButton::clicked, this, &TrajectoryTab::computeJointPath);
  connect(trajectory_button_, &QPushButton::clicked, this, &TrajectoryTab::computeTrajectory);
  connect(execute_button_, &QPushButton::clicked, this, &TrajectoryTab::executeTrajectory);
  connect(cancel_button_, &QPushButton::clicked, this, &TrajectoryTab::cancel);
  connect(joint_states_topic_, &QLineEdit::editingFinished, this, [this]() {
      subscribeJointStates();
    });

  for (auto edit : {raster_node_, joint_path_action_, time_parametrization_action_,
      execute_action_, joint_states_topic_, mesh_file_, mesh_frame_, object_file_, object_frame_,
      planning_frame_, group_name_})
  {
    connect(edit, &QLineEdit::editingFinished, this, &TrajectoryTab::configChanged);
  }
  for (auto spin : {padding_, row_step_, row_angle_, edge_margin_, deviation_[0], deviation_[1], deviation_[2], scaling_factor_}) {
    connect(
      spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
      &TrajectoryTab::configChanged);
  }
  connect(keep_tool_orientation_, &QCheckBox::toggled, this, &TrajectoryTab::configChanged);

  // The joint path depends on the planning frame and on the deviation: drop the old one
  connect(planning_frame_, &QLineEdit::textEdited, this, [this]() {invalidateFrom(1);});
  for (auto spin : deviation_) {
    connect(
      spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
      [this]() {invalidateFrom(1);});
  }

  invalidateFrom(0);
}

void TrajectoryTab::initialize(const rclcpp::Node::SharedPtr & node)
{
  TabBase::initialize(node);
  display_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    "~/display_trajectory", rclcpp::QoS(1).transient_local());
  subscribeJointStates();
}

void TrajectoryTab::subscribeJointStates()
{
  if (!node_) {
    return;
  }
  const std::string topic = joint_states_topic_->text().trimmed().toStdString();
  if (joint_states_sub_ && topic == joint_states_sub_->get_topic_name()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    joint_state_.reset();
  }
  // Best effort QoS: it matches both reliable and best effort publishers
  joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      joint_state_ = msg;
    });
}

void TrajectoryTab::setMeshFile(const QString & filename)
{
  mesh_file_->setText(filename);
  Q_EMIT configChanged();
}

void TrajectoryTab::setObjectFile(const QString & filename)
{
  object_file_->setText(filename);
  Q_EMIT configChanged();
}

void TrajectoryTab::updateButtons()
{
  const bool idle = !busy();
  load_topic_button_->setEnabled(idle);
  load_file_button_->setEnabled(idle);
  raster_button_->setEnabled(idle);
  joint_path_button_->setEnabled(idle && raster_ && !raster_->poses.poses.empty());
  trajectory_button_->setEnabled(idle && joint_path_ && !joint_path_->trajectory.points.empty());
  execute_button_->setEnabled(idle && trajectory_ != nullptr);
  cancel_button_->setEnabled(hasRunningGoal());
}

void TrajectoryTab::invalidateFrom(int step)
{
  if (step <= 0) {
    raster_.reset();
    raster_label_->setText("No raster path");
  }
  if (step <= 1) {
    joint_path_.reset();
    joint_path_label_->setText("No joint path");
  }
  if (step <= 2) {
    trajectory_.reset();
    trajectory_label_->setText("No trajectory");
  }
  updateButtons();
}

void TrajectoryTab::setProgress(int value, int maximum, const QString & format)
{
  progress_->setRange(0, maximum);  // maximum = 0: busy indicator
  progress_->setValue(value);
  progress_->setFormat(format);
  progress_->setVisible(true);
}

void TrajectoryTab::hideProgress()
{
  progress_->setVisible(false);
}

void TrajectoryTab::cancel()
{
  cancelRunningGoal();
}

// ------------------------------------------------------------------------------------- 1. Mesh

void TrajectoryTab::loadMeshFromTopic()
{
  using Trigger = std_srvs::srv::Trigger;
  const bool sent = callService<Trigger>(
    joinName(raster_node_->text(), "load_mesh_from_topic"), std::make_shared<Trigger::Request>(),
    kLoadTimeoutMs, [this](Trigger::Response::SharedPtr response) {
      setBusy(false);
      if (!response) {
        return;
      }
      const QString message = QString::fromStdString(response->message);
      if (!response->success) {
        error(message);
        return;
      }
      info(message);
      mesh_label_->setText("From topic: " + message);
      invalidateFrom(0);
    });
  if (sent) {
    setBusy(true);
  }
}

void TrajectoryTab::loadMeshFromFile()
{
  using LoadMesh = surface_raster_planner_msgs::srv::LoadMesh;
  const QString filename = mesh_file_->text().trimmed();
  if (filename.isEmpty()) {
    error("Select a mesh file first");
    return;
  }
  auto request = std::make_shared<LoadMesh::Request>();
  request->filename = filename.toStdString();
  request->frame_id = mesh_frame_->text().trimmed().toStdString();

  const bool sent = callService<LoadMesh>(
    joinName(raster_node_->text(), "load_mesh_from_file"), request, kLoadTimeoutMs,
    [this, filename](LoadMesh::Response::SharedPtr response) {
      setBusy(false);
      if (!response) {
        return;
      }
      const QString message = QString::fromStdString(response->message);
      if (!response->success) {
        error(message);
        return;
      }
      info(message);
      mesh_label_->setText(QString("From %1: %2").arg(filename, message));
      invalidateFrom(0);
    });
  if (sent) {
    setBusy(true);
  }
}

void TrajectoryTab::browseMeshFile()
{
  const QString start = mesh_file_->text().isEmpty() ? QDir::homePath() : mesh_file_->text();
  const QString file = QFileDialog::getOpenFileName(
    this, "Load mesh", start, "Meshes (*.ply *.obj *.vtk *.stl);;All files (*)");
  if (!file.isEmpty()) {
    setMeshFile(file);
  }
}

// ------------------------------------------------------------------------------ 2. Collision object

void TrajectoryTab::loadObjectFromFile()
{
  using LoadObject = scene_tools_interfaces::srv::AddObject;
  const QString filename = object_file_->text().trimmed();
  if (filename.isEmpty()) {
    error("Select an object file first");
    return;
  }
  auto request = std::make_shared<LoadObject::Request>();
  request->mesh_file = filename.toStdString();
  request->frame_id = object_frame_->text().trimmed().isEmpty()
    ? "table"
    : object_frame_->text().trimmed().toStdString();
  request->scale = 1.0;
  request->padding = padding_->value();

  const bool sent = callService<LoadObject>(
    "/add_object", request, kLoadTimeoutMs,
    [this, filename](LoadObject::Response::SharedPtr response) {
      setBusy(false);
      if (!response) {
        return;
      }
      const QString message = QString::fromStdString(response->message);
      if (!response->success) {
        error(message);
        return;
      }
      info(message);
      object_label_->setText(QString("From %1: %2").arg(filename, message));
      invalidateFrom(0);
    });
  if (sent) {
    setBusy(true);
  }
}

void TrajectoryTab::browseObjectFile()
{
  const QString start = object_file_->text().isEmpty() ? QDir::homePath() : object_file_->text();
  const QString file = QFileDialog::getOpenFileName(
    this, "Load object", start, "objectes (*.ply *.obj *.vtk *.stl);;All files (*)");
  if (!file.isEmpty()) {
    setObjectFile(file);
  }
}



// ------------------------------------------------------------------------------ 3. Raster path

void TrajectoryTab::computeRaster()
{
  auto request = std::make_shared<ComputeTrajectory::Request>();
  request->row_step = row_step_->value();
  request->keep_tool_orientation = keep_tool_orientation_->isChecked();
  request->row_angle = qDegreesToRadians(row_angle_->value());
  request->edge_margin = edge_margin_->value();

  const bool sent = callService<ComputeTrajectory>(
    joinName(raster_node_->text(), "compute_trajectory"), request, kRasterTimeoutMs,
    [this](ComputeTrajectory::Response::SharedPtr response) {
      setBusy(false);
      if (!response) {
        return;
      }
      const QString message = QString::fromStdString(response->message);
      if (!response->success) {
        error(message);
        return;
      }
      info(message);
      invalidateFrom(0);
      raster_ = response;
      raster_label_->setText(
        QString("%1 poses, %2 segments, %3 raster points discarded, %4 within the edge margin")
        .arg(response->poses.poses.size())
        .arg(response->poses.poses.empty() ? 0 : response->jump_indices.size() + 1)
        .arg(response->discarded_points)
        .arg(response->edge_discarded_points));
      if (response->poses.poses.empty()) {
        warn("The raster path is empty");
      }
      updateButtons();
    });
  if (sent) {
    setBusy(true);
  }
}

// ------------------------------------------------------------------------------- 4. Joint path

void TrajectoryTab::computeJointPath()
{
  if (!raster_) {
    return;
  }
  ComputeJointPath::Goal goal;
  goal.poses = raster_->poses;
  goal.jump_indices = raster_->jump_indices;
  goal.planning_frame = planning_frame_->text().trimmed().toStdString();
  goal.deviation_distance.x = deviation_[0]->value();
  goal.deviation_distance.y = deviation_[1]->value();
  goal.deviation_distance.z = deviation_[2]->value();

  const bool sent = sendGoal<ComputeJointPath>(
    joint_path_action_->text().trimmed().toStdString(), goal,
    [this](const ComputeJointPath::Feedback & feedback) {
      setProgress(
        static_cast<int>(feedback.current_index), static_cast<int>(feedback.total_poses),
        QString("%v/%m  ") + QString::fromStdString(feedback.status));
    },
    [this](rclcpp_action::ResultCode code, ComputeJointPath::Result::SharedPtr result) {
      hideProgress();
      setBusy(false);
      if (!result) {
        return;  // rejected or cancelled, already logged
      }
      const QString message = QString::fromStdString(result->message);
      if (code != rclcpp_action::ResultCode::SUCCEEDED || !result->success) {
        error("Joint path failed: " + message);
        return;
      }
      info(message);
      invalidateFrom(1);
      joint_path_ = result;
      int reached = 0;
      for (int i : result->pose_indices) {
        reached += i >= 0;
      }
      joint_path_label_->setText(
        QString("%1 points, %2 poses reached, %3 skipped")
        .arg(result->trajectory.points.size()).arg(reached).arg(result->skipped_indices.size()));
      if (!result->skipped_indices.empty()) {
        warn(QString("%1 poses skipped (no feasible configuration)")
          .arg(result->skipped_indices.size()));
      }
      updateButtons();
    });
  if (sent) {
    setProgress(0, 0, "Computing joint path...");
    setBusy(true);
  }
}

// ------------------------------------------------------------------------------- 5. Trajectory

void TrajectoryTab::computeTrajectory()
{
  using ApplyTimeParametrization = time_parametrization_msgs::action::ApplyTimeParametrization;
  using moveit_msgs::msg::MoveItErrorCodes;
  if (!joint_path_ || joint_path_->trajectory.points.empty()) {
    return;
  }

  // The joint path does not contain the current configuration: prepend it, otherwise MoveIt
  // rejects the trajectory because its first point is not the robot state.
  sensor_msgs::msg::JointState::SharedPtr joint_state;
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    joint_state = joint_state_;
  }
  if (!joint_state) {
    error(QString("No message received on %1").arg(joint_states_topic_->text()));
    return;
  }
  std::map<std::string, double> current;
  for (size_t i = 0; i < joint_state->name.size() && i < joint_state->position.size(); ++i) {
    current[joint_state->name[i]] = joint_state->position[i];
  }
  trajectory_msgs::msg::JointTrajectory path = joint_path_->trajectory;
  std::vector<double> q_now;
  for (const auto & joint : path.joint_names) {
    auto it = current.find(joint);
    if (it == current.end()) {
      error(QString("Joint %1 not in %2").arg(
          QString::fromStdString(joint), joint_states_topic_->text()));
      return;
    }
    q_now.push_back(it->second);
  }
  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.positions = q_now;
  path.points.insert(path.points.begin(), start_point);

  moveit_msgs::msg::RobotState start_state;
  start_state.joint_state.name = path.joint_names;
  start_state.joint_state.position = q_now;

  ApplyTimeParametrization::Goal goal;
  auto & plan = goal.unparametrized_motion_plan_response;
  plan.group_name = group_name_->text().trimmed().toStdString();
  plan.trajectory.joint_trajectory = path;
  plan.trajectory_start = start_state;
  goal.max_velocity_scaling_factor = scaling_factor_->value();
  goal.max_acceleration_scaling_factor = scaling_factor_->value();

  const double scaling = scaling_factor_->value();
  const bool sent = sendGoal<ApplyTimeParametrization>(
    time_parametrization_action_->text().trimmed().toStdString(), goal, nullptr,
    [this, start_state, scaling](
      rclcpp_action::ResultCode code, ApplyTimeParametrization::Result::SharedPtr result) {
      hideProgress();
      setBusy(false);
      if (!result) {
        return;
      }
      const auto & response = result->motion_plan_response;
      if (code != rclcpp_action::ResultCode::SUCCEEDED ||
      response.error_code.val != MoveItErrorCodes::SUCCESS)
      {
        error(QString("Time parametrization failed (error code %1)").arg(response.error_code.val));
        return;
      }
      const auto & points = response.trajectory.joint_trajectory.points;
      if (points.empty()) {
        error("Time parametrization returned an empty trajectory");
        return;
      }
      invalidateFrom(2);
      trajectory_ = std::make_shared<moveit_msgs::msg::RobotTrajectory>(response.trajectory);
      const double duration = seconds(points.back().time_from_start);
      const QString text = QString("%1 points, %2 s (scaling factor %3)")
        .arg(points.size()).arg(duration, 0, 'f', 2).arg(scaling);
      trajectory_label_->setText(text);
      info("Trajectory: " + text);

      moveit_msgs::msg::DisplayTrajectory display;
      display.trajectory_start = start_state;
      display.trajectory.push_back(*trajectory_);
      display_pub_->publish(display);
      updateButtons();
    });
  if (sent) {
    setProgress(0, 0, "Time parametrization...");
    setBusy(true);
  }
}

void TrajectoryTab::executeTrajectory()
{
  using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
  using moveit_msgs::msg::MoveItErrorCodes;
  if (!trajectory_) {
    return;
  }
  const double duration =
    seconds(trajectory_->joint_trajectory.points.back().time_from_start);
  const auto answer = QMessageBox::warning(
    this, "Execute trajectory",
    QString("The robot will move along the trajectory (%1 s).\nContinue?")
    .arg(duration, 0, 'f', 1),
    QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  if (answer != QMessageBox::Yes) {
    return;
  }

  ExecuteTrajectory::Goal goal;
  goal.trajectory = *trajectory_;
  const bool sent = sendGoal<ExecuteTrajectory>(
    execute_action_->text().trimmed().toStdString(), goal, nullptr,
    [this](rclcpp_action::ResultCode code, ExecuteTrajectory::Result::SharedPtr result) {
      hideProgress();
      setBusy(false);
      // The robot is no longer in the start configuration of the trajectory
      invalidateFrom(2);
      if (!result) {
        return;
      }
      if (code != rclcpp_action::ResultCode::SUCCEEDED ||
      result->error_code.val != MoveItErrorCodes::SUCCESS)
      {
        error(QString("Execution failed (error code %1)").arg(result->error_code.val));
        return;
      }
      info("Trajectory executed");
    });
  if (sent) {
    setProgress(0, 0, "Executing...");
    setBusy(true);
  }
}

// ------------------------------------------------------------------------------------- Config

void TrajectoryTab::save(rviz_common::Config config) const
{
  config.mapSetValue("raster_node", raster_node_->text());
  config.mapSetValue("joint_path_action", joint_path_action_->text());
  config.mapSetValue("time_parametrization_action", time_parametrization_action_->text());
  config.mapSetValue("execute_action", execute_action_->text());
  config.mapSetValue("joint_states_topic", joint_states_topic_->text());
  config.mapSetValue("mesh_file", mesh_file_->text());
  config.mapSetValue("mesh_frame", mesh_frame_->text());
  config.mapSetValue("object_file", object_file_->text());
  config.mapSetValue("object_frame", object_frame_->text());
  config.mapSetValue("padding", padding_->value());
  config.mapSetValue("row_step", row_step_->value());
  config.mapSetValue("row_angle_deg", row_angle_->value());
  config.mapSetValue("edge_margin", edge_margin_->value());
  config.mapSetValue("keep_tool_orientation", keep_tool_orientation_->isChecked());
  config.mapSetValue("planning_frame", planning_frame_->text());
  config.mapSetValue("deviation_x", deviation_[0]->value());
  config.mapSetValue("deviation_y", deviation_[1]->value());
  config.mapSetValue("deviation_z", deviation_[2]->value());
  config.mapSetValue("group_name", group_name_->text());
  config.mapSetValue("scaling_factor", scaling_factor_->value());
}

void TrajectoryTab::load(const rviz_common::Config & config)
{
  const std::pair<const char *, QLineEdit *> strings[] = {
    {"raster_node", raster_node_},
    {"joint_path_action", joint_path_action_},
    {"time_parametrization_action", time_parametrization_action_},
    {"execute_action", execute_action_},
    {"joint_states_topic", joint_states_topic_},
    {"mesh_file", mesh_file_},
    {"mesh_frame", mesh_frame_},
    {"object_file", object_file_},
    {"object_frame", object_frame_},
    {"planning_frame", planning_frame_},
    {"group_name", group_name_},
  };
  for (const auto & [key, edit] : strings) {
    QString value;
    if (config.mapGetString(key, &value)) {
      edit->setText(value);
    }
  }
  const std::pair<const char *, QDoubleSpinBox *> numbers[] = {
    {"padding", padding_},
    {"row_step", row_step_},
    {"row_angle_deg", row_angle_},
    {"edge_margin", edge_margin_},
    {"deviation_x", deviation_[0]},
    {"deviation_y", deviation_[1]},
    {"deviation_z", deviation_[2]},
    {"scaling_factor", scaling_factor_},
  };
  for (const auto & [key, spin] : numbers) {
    float value;
    if (config.mapGetFloat(key, &value)) {
      spin->setValue(value);
    }
  }
  bool keep;
  if (config.mapGetBool("keep_tool_orientation", &keep)) {
    keep_tool_orientation_->setChecked(keep);
  }
  subscribeJointStates();
}

}  // namespace surface_gui
