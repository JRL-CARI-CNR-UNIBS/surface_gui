// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#ifndef SURFACE_GUI__RECONSTRUCTION_TAB_HPP_
#define SURFACE_GUI__RECONSTRUCTION_TAB_HPP_

#include <QString>

#include "surface_gui/tab_base.hpp"

class QLabel;
class QLineEdit;
class QPushButton;

namespace surface_gui
{

/// Tab "Acquisition": drives cloud_merger_node (package surface_reconstruction).
///
/// acquire, remove last, reset, compute surface and save mesh.
class ReconstructionTab : public TabBase
{
  Q_OBJECT

public:
  explicit ReconstructionTab(QWidget * parent = nullptr);

  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

Q_SIGNALS:
  /// The mesh was saved to this (absolute) file.
  void meshSaved(const QString & filename);

protected:
  void updateButtons() override;

private:
  void acquire();
  void removeLast();
  void reset();
  void computeSurface();
  void saveMesh();
  void browseFile();

  /// Call a std_srvs/Trigger service of the node; on success the count is parsed from the message.
  void callTrigger(const char * service, int timeout_ms, const QString & count_pattern);
  void setAcquisitions(int n);

  QLineEdit * node_name_;
  QPushButton * acquire_button_;
  QPushButton * remove_last_button_;
  QPushButton * reset_button_;
  QPushButton * compute_button_;
  QPushButton * save_button_;
  QPushButton * browse_button_;
  QLineEdit * filename_;
  QLabel * acquisitions_label_;
  QLabel * surface_label_;
};

}  // namespace surface_gui

#endif  // SURFACE_GUI__RECONSTRUCTION_TAB_HPP_
