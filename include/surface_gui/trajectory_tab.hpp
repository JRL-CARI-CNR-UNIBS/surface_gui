// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#ifndef SURFACE_GUI__TRAJECTORY_TAB_HPP_
#define SURFACE_GUI__TRAJECTORY_TAB_HPP_

#include <memory>
#include <mutex>

#include <QString>

#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <surface_path_planning_msgs/action/compute_joint_path.hpp>
#include <surface_raster_planner_msgs/srv/compute_trajectory.hpp>

#include "surface_gui/tab_base.hpp"

class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;

namespace surface_gui
{

/// Tab "Path & trajectory": the chain after the reconstruction.
///
/// 1. load the mesh in raster_planner_node (from topic or file);
/// 2. raster path (compute_trajectory service of surface_raster_planner);
/// 3. joint path (compute_joint_path action of surface_path_planning);
/// 4. time parametrization (action of the time_parametrization package);
/// 5. execution with MoveIt (execute_trajectory action of move_group).
///
/// Each step uses the result of the previous one; a new result invalidates the following ones.
class TrajectoryTab : public TabBase
{
  Q_OBJECT

public:
  explicit TrajectoryTab(QWidget * parent = nullptr);

  void initialize(const rclcpp::Node::SharedPtr & node) override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

public Q_SLOTS:
  /// Set the file used by "Load from file" (e.g. the mesh just saved by the reconstruction tab).
  void setMeshFile(const QString & filename);

protected:
  void updateButtons() override;

private:
  using ComputeTrajectory = surface_raster_planner_msgs::srv::ComputeTrajectory;
  using ComputeJointPath = surface_path_planning_msgs::action::ComputeJointPath;

  void loadMeshFromTopic();
  void loadMeshFromFile();
  void browseMeshFile();
  void computeRaster();
  void computeJointPath();
  void computeTrajectory();
  void executeTrajectory();
  void cancel();

  /// (Re)create the joint states subscription with the topic in the interfaces section.
  void subscribeJointStates();

  /// Drop the results from the given step on (0 raster, 1 joint path, 2 trajectory).
  void invalidateFrom(int step);
  void setProgress(int value, int maximum, const QString & format);
  void hideProgress();

  // Interfaces
  QLineEdit * raster_node_;
  QLineEdit * joint_path_action_;
  QLineEdit * time_parametrization_action_;
  QLineEdit * execute_action_;
  QLineEdit * joint_states_topic_;

  // 1. Mesh
  QPushButton * load_topic_button_;
  QLineEdit * mesh_file_;
  QPushButton * browse_button_;
  QLineEdit * mesh_frame_;
  QPushButton * load_file_button_;
  QLabel * mesh_label_;

  // 2. Raster path
  QDoubleSpinBox * row_step_;
  QCheckBox * keep_tool_orientation_;
  QPushButton * raster_button_;
  QLabel * raster_label_;

  // 3. Joint path
  QLineEdit * planning_frame_;
  QDoubleSpinBox * deviation_[3];
  QPushButton * joint_path_button_;
  QLabel * joint_path_label_;

  // 4. Trajectory
  QLineEdit * group_name_;
  QDoubleSpinBox * scaling_factor_;
  QPushButton * trajectory_button_;
  QPushButton * execute_button_;
  QLabel * trajectory_label_;

  QProgressBar * progress_;
  QPushButton * cancel_button_;

  // Results
  std::shared_ptr<ComputeTrajectory::Response> raster_;
  std::shared_ptr<ComputeJointPath::Result> joint_path_;
  std::shared_ptr<moveit_msgs::msg::RobotTrajectory> trajectory_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState::SharedPtr joint_state_;  ///< protected by joint_state_mutex_
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
};

}  // namespace surface_gui

#endif  // SURFACE_GUI__TRAJECTORY_TAB_HPP_
