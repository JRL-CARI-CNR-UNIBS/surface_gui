// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#ifndef SURFACE_GUI__SURFACE_PANEL_HPP_
#define SURFACE_GUI__SURFACE_PANEL_HPP_

#include <memory>
#include <thread>

#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

class QPlainTextEdit;
class QTabWidget;

namespace surface_gui
{

class ReconstructionTab;
class TrajectoryTab;

/// Rviz 2 panel of the surface pipeline.
///
/// Tab "Acquisition": acquire / remove last / reset / compute surface / save (cloud_merger_node).
/// Tab "Path & trajectory": mesh loading, raster path, joint path, time parametrization and
/// execution. A log at the bottom shows the messages of both tabs.
///
/// The panel owns a ROS node ("surface_gui_panel") spun in a separate thread, so the calls to
/// services and actions never block Rviz.
class SurfacePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SurfacePanel(QWidget * parent = nullptr);
  ~SurfacePanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void appendLog(int level, const QString & text);

private:
  QTabWidget * tabs_;
  ReconstructionTab * reconstruction_tab_;
  TrajectoryTab * trajectory_tab_;
  QPlainTextEdit * log_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
};

}  // namespace surface_gui

#endif  // SURFACE_GUI__SURFACE_PANEL_HPP_
