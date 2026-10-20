// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include "surface_gui/reconstruction_tab.hpp"

#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QRegularExpression>
#include <QVBoxLayout>

#include <std_srvs/srv/trigger.hpp>
#include <surface_reconstruction_msgs/srv/save_mesh.hpp>

namespace surface_gui
{

namespace
{
constexpr int kTriggerTimeoutMs = 30000;
constexpr int kComputeTimeoutMs = 600000;  // Poisson on big clouds can be slow
constexpr int kSaveTimeoutMs = 60000;
}  // namespace

ReconstructionTab::ReconstructionTab(QWidget * parent)
: TabBase(parent)
{
  // --- Acquisition
  acquire_button_ = new QPushButton("Acquire");
  acquire_button_->setToolTip(
    "Store the latest filtered cloud, moved to the common frame with the current robot pose");
  remove_last_button_ = new QPushButton("Remove last");
  remove_last_button_->setToolTip("Remove the last acquisition (e.g. robot not yet still)");
  reset_button_ = new QPushButton("Reset");
  reset_button_->setToolTip("Clear all the acquisitions and the mesh");
  acquisitions_label_ = new QLabel;

  auto acquisition_box = new QGroupBox("Acquisition");
  auto acquisition_layout = new QVBoxLayout(acquisition_box);
  auto acquisition_buttons = new QHBoxLayout;
  acquisition_buttons->addWidget(acquire_button_);
  acquisition_buttons->addWidget(remove_last_button_);
  acquisition_buttons->addWidget(reset_button_);
  acquisition_layout->addLayout(acquisition_buttons);
  acquisition_layout->addWidget(acquisitions_label_);

  // --- Reconstruction
  compute_button_ = new QPushButton("Compute surface");
  compute_button_->setToolTip("Merge the acquisitions and reconstruct the mesh (Poisson)");
  surface_label_ = new QLabel("No surface computed");
  surface_label_->setWordWrap(true);

  auto surface_box = new QGroupBox("Reconstruction");
  auto surface_layout = new QVBoxLayout(surface_box);
  surface_layout->addWidget(compute_button_);
  surface_layout->addWidget(surface_label_);

  // --- Save
  filename_ = new QLineEdit;
  filename_->setPlaceholderText("empty: node parameter default_mesh_file");
  filename_->setToolTip("Output file; the extension selects the format: .ply, .vtk, .obj");
  browse_button_ = new QPushButton("...");
  browse_button_->setMaximumWidth(32);
  save_button_ = new QPushButton("Save mesh");

  auto save_box = new QGroupBox("Save");
  auto save_layout = new QVBoxLayout(save_box);
  auto file_row = new QHBoxLayout;
  file_row->addWidget(filename_);
  file_row->addWidget(browse_button_);
  save_layout->addLayout(file_row);
  save_layout->addWidget(save_button_);

  // --- Interfaces
  node_name_ = new QLineEdit("/cloud_merger_node");
  auto interfaces = new QWidget;
  auto interfaces_layout = new QFormLayout(interfaces);
  interfaces_layout->addRow("Reconstruction node", node_name_);

  auto layout = new QVBoxLayout(this);
  layout->addWidget(acquisition_box);
  layout->addWidget(surface_box);
  layout->addWidget(save_box);
  layout->addWidget(makeCollapsible("Interfaces", interfaces));
  layout->addStretch();

  connect(acquire_button_, &QPushButton::clicked, this, &ReconstructionTab::acquire);
  connect(remove_last_button_, &QPushButton::clicked, this, &ReconstructionTab::removeLast);
  connect(reset_button_, &QPushButton::clicked, this, &ReconstructionTab::reset);
  connect(compute_button_, &QPushButton::clicked, this, &ReconstructionTab::computeSurface);
  connect(save_button_, &QPushButton::clicked, this, &ReconstructionTab::saveMesh);
  connect(browse_button_, &QPushButton::clicked, this, &ReconstructionTab::browseFile);
  connect(node_name_, &QLineEdit::editingFinished, this, &ReconstructionTab::configChanged);
  connect(filename_, &QLineEdit::editingFinished, this, &ReconstructionTab::configChanged);

  setAcquisitions(-1);
  updateButtons();
}

void ReconstructionTab::updateButtons()
{
  const bool idle = !busy();
  for (auto button : {acquire_button_, remove_last_button_, reset_button_, compute_button_,
      save_button_})
  {
    button->setEnabled(idle);
  }
}

void ReconstructionTab::setAcquisitions(int n)
{
  acquisitions_label_->setText(
    n < 0 ? QString("Acquisitions: -") : QString("Acquisitions: %1").arg(n));
}

void ReconstructionTab::callTrigger(
  const char * service, int timeout_ms, const QString & count_pattern)
{
  using Trigger = std_srvs::srv::Trigger;
  const std::string name = joinName(node_name_->text(), service);
  const bool sent = callService<Trigger>(
    name, std::make_shared<Trigger::Request>(), timeout_ms,
    [this, count_pattern, service](Trigger::Response::SharedPtr response) {
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
      if (!count_pattern.isEmpty()) {
        const auto match = QRegularExpression(count_pattern).match(message);
        if (match.hasMatch()) {
          setAcquisitions(match.captured(1).toInt());
        }
      }
      const std::string srv(service);
      if (srv == "compute_surface") {
        surface_label_->setText(message);
      } else if (srv == "reset") {
        setAcquisitions(0);
        surface_label_->setText("No surface computed");
      }
    });
  if (sent) {
    setBusy(true);
  }
}

void ReconstructionTab::acquire()
{
  callTrigger("acquire", kTriggerTimeoutMs, "Acquisition #(\\d+)");
}

void ReconstructionTab::removeLast()
{
  callTrigger("remove_last", kTriggerTimeoutMs, "(\\d+) remaining");
}

void ReconstructionTab::reset()
{
  if (QMessageBox::question(
      this, "Reset", "Clear all the acquisitions and the mesh?") != QMessageBox::Yes)
  {
    return;
  }
  callTrigger("reset", kTriggerTimeoutMs, QString());
}

void ReconstructionTab::computeSurface()
{
  info("Computing the surface...");
  callTrigger("compute_surface", kComputeTimeoutMs, "from (\\d+) acquisitions");
}

void ReconstructionTab::saveMesh()
{
  using SaveMesh = surface_reconstruction_msgs::srv::SaveMesh;
  auto request = std::make_shared<SaveMesh::Request>();
  const QString filename = filename_->text().trimmed();
  request->filename = filename.toStdString();

  const bool sent = callService<SaveMesh>(
    joinName(node_name_->text(), "save"), request, kSaveTimeoutMs,
    [this, filename](SaveMesh::Response::SharedPtr response) {
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
      // A relative path is resolved by the node, whose working directory is unknown here
      if (!filename.isEmpty() && QFileInfo(filename).isAbsolute()) {
        Q_EMIT meshSaved(filename);
      }
    });
  if (sent) {
    setBusy(true);
  }
}

void ReconstructionTab::browseFile()
{
  const QString start = filename_->text().isEmpty() ? QDir::homePath() : filename_->text();
  const QString file = QFileDialog::getSaveFileName(
    this, "Save mesh", start, "Meshes (*.ply *.vtk *.obj);;All files (*)");
  if (!file.isEmpty()) {
    filename_->setText(file);
    Q_EMIT configChanged();
  }
}

void ReconstructionTab::save(rviz_common::Config config) const
{
  config.mapSetValue("node", node_name_->text());
  config.mapSetValue("mesh_file", filename_->text());
}

void ReconstructionTab::load(const rviz_common::Config & config)
{
  QString value;
  if (config.mapGetString("node", &value)) {
    node_name_->setText(value);
  }
  if (config.mapGetString("mesh_file", &value)) {
    filename_->setText(value);
  }
}

}  // namespace surface_gui
