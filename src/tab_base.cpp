// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include "surface_gui/tab_base.hpp"

#include <QToolButton>
#include <QVBoxLayout>

namespace surface_gui
{

TabBase::TabBase(QWidget * parent)
: QWidget(parent)
{
}

void TabBase::initialize(const rclcpp::Node::SharedPtr & node)
{
  node_ = node;
}

void TabBase::setBusy(bool busy)
{
  busy_ = busy;
  updateButtons();
}

void TabBase::cancelRunningGoal()
{
  if (!cancel_) {
    return;
  }
  // Move it out first: the call resets cancel_ while it is running.
  auto cancel = std::move(cancel_);
  cancel_ = nullptr;
  cancel();
}

bool TabBase::checkNode()
{
  if (!node_) {
    error("ROS node not initialized");
    return false;
  }
  return true;
}

QWidget * TabBase::makeCollapsible(const QString & title, QWidget * content)
{
  auto widget = new QWidget;
  auto layout = new QVBoxLayout(widget);
  layout->setContentsMargins(0, 0, 0, 0);

  auto button = new QToolButton;
  button->setText(title);
  button->setCheckable(true);
  button->setChecked(false);
  button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  button->setArrowType(Qt::RightArrow);
  button->setAutoRaise(true);

  content->setVisible(false);
  QObject::connect(
    button, &QToolButton::toggled, content, [button, content](bool checked) {
      button->setArrowType(checked ? Qt::DownArrow : Qt::RightArrow);
      content->setVisible(checked);
    });

  layout->addWidget(button);
  layout->addWidget(content);
  return widget;
}

std::string TabBase::joinName(const QString & node, const char * name)
{
  QString prefix = node.trimmed();
  while (prefix.endsWith('/')) {
    prefix.chop(1);
  }
  return (prefix + "/" + name).toStdString();
}

}  // namespace surface_gui
