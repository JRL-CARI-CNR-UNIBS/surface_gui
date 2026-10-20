// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include "surface_gui/surface_panel.hpp"

#include <QDateTime>
#include <QFrame>
#include <QHBoxLayout>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QTabWidget>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

#include "surface_gui/reconstruction_tab.hpp"
#include "surface_gui/trajectory_tab.hpp"

namespace surface_gui
{

namespace
{
/// The tabs are long: put them in a scroll area so the panel also fits a small dock.
QScrollArea * scrollable(QWidget * widget)
{
  auto area = new QScrollArea;
  area->setWidget(widget);
  area->setWidgetResizable(true);
  area->setFrameShape(QFrame::NoFrame);
  return area;
}
}  // namespace

SurfacePanel::SurfacePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  reconstruction_tab_ = new ReconstructionTab;
  trajectory_tab_ = new TrajectoryTab;

  tabs_ = new QTabWidget;
  tabs_->addTab(scrollable(reconstruction_tab_), "Acquisition");
  tabs_->addTab(scrollable(trajectory_tab_), "Path && trajectory");

  log_ = new QPlainTextEdit;
  log_->setReadOnly(true);
  log_->setMaximumBlockCount(500);
  log_->setPlaceholderText("Log");
  auto clear_button = new QPushButton("Clear log");

  auto log_widget = new QWidget;
  auto log_layout = new QVBoxLayout(log_widget);
  log_layout->setContentsMargins(0, 0, 0, 0);
  log_layout->addWidget(log_);
  auto clear_row = new QHBoxLayout;
  clear_row->addStretch();
  clear_row->addWidget(clear_button);
  log_layout->addLayout(clear_row);

  auto splitter = new QSplitter(Qt::Vertical);
  splitter->addWidget(tabs_);
  splitter->addWidget(log_widget);
  splitter->setStretchFactor(0, 4);
  splitter->setStretchFactor(1, 1);

  auto layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(splitter);

  connect(clear_button, &QPushButton::clicked, log_, &QPlainTextEdit::clear);
  connect(tabs_, &QTabWidget::currentChanged, this, &SurfacePanel::configChanged);
  for (TabBase * tab : std::initializer_list<TabBase *>{reconstruction_tab_, trajectory_tab_}) {
    connect(tab, &TabBase::logMessage, this, &SurfacePanel::appendLog);
    connect(tab, &TabBase::configChanged, this, &SurfacePanel::configChanged);
  }
  connect(
    reconstruction_tab_, &ReconstructionTab::meshSaved, trajectory_tab_,
    &TrajectoryTab::setMeshFile);
}

SurfacePanel::~SurfacePanel()
{
  // Stop the ROS callbacks before the widgets they refer to are destroyed
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void SurfacePanel::onInitialize()
{
  node_ = std::make_shared<rclcpp::Node>("surface_gui_panel");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {executor_->spin();});

  reconstruction_tab_->initialize(node_);
  trajectory_tab_->initialize(node_);
}

void SurfacePanel::appendLog(int level, const QString & text)
{
  const char * color = "";
  switch (level) {
    case TabBase::WARN:
      color = "#d08000";
      if (node_) {RCLCPP_WARN(node_->get_logger(), "%s", text.toStdString().c_str());}
      break;
    case TabBase::ERROR:
      color = "#d00000";
      if (node_) {RCLCPP_ERROR(node_->get_logger(), "%s", text.toStdString().c_str());}
      break;
    default:
      if (node_) {RCLCPP_INFO(node_->get_logger(), "%s", text.toStdString().c_str());}
      break;
  }
  const QString time = QDateTime::currentDateTime().toString("HH:mm:ss");
  const QString body = text.toHtmlEscaped();
  log_->appendHtml(
    *color ?
    QString("%1 <span style=\"color:%2\">%3</span>").arg(time, color, body) :
    QString("%1 %2").arg(time, body));
}

void SurfacePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("current_tab", tabs_->currentIndex());
  reconstruction_tab_->save(config.mapMakeChild("acquisition"));
  trajectory_tab_->save(config.mapMakeChild("trajectory"));
}

void SurfacePanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  reconstruction_tab_->load(config.mapGetChild("acquisition"));
  trajectory_tab_->load(config.mapGetChild("trajectory"));
  int index;
  if (config.mapGetInt("current_tab", &index)) {
    tabs_->setCurrentIndex(index);
  }
}

}  // namespace surface_gui

PLUGINLIB_EXPORT_CLASS(surface_gui::SurfacePanel, rviz_common::Panel)
