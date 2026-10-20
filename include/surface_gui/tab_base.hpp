// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#ifndef SURFACE_GUI__TAB_BASE_HPP_
#define SURFACE_GUI__TAB_BASE_HPP_

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <typeinfo>
#include <utility>

#include <QMetaObject>
#include <QString>
#include <QTimer>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/config.hpp>

namespace surface_gui
{

/// Base class of the panel tabs: ROS service and action calls that never block the GUI.
///
/// The ROS node is spun by the panel in a separate thread. All the callbacks given to
/// callService() and sendGoal() are executed in the Qt (GUI) thread, so they can update the
/// widgets directly. Only one action goal per tab can be running at a time.
class TabBase : public QWidget
{
  Q_OBJECT

public:
  enum Level { INFO = 0, WARN = 1, ERROR = 2 };

  explicit TabBase(QWidget * parent = nullptr);
  ~TabBase() override = default;

  /// Called once by the panel, after the ROS node is created.
  virtual void initialize(const rclcpp::Node::SharedPtr & node);

  /// Store and restore the widget values in the Rviz config.
  virtual void save(rviz_common::Config config) const = 0;
  virtual void load(const rviz_common::Config & config) = 0;

Q_SIGNALS:
  /// A message for the panel log.
  void logMessage(int level, const QString & text);
  /// A value stored in the Rviz config changed.
  void configChanged();

protected:
  void info(const QString & text) {Q_EMIT logMessage(INFO, text);}
  void warn(const QString & text) {Q_EMIT logMessage(WARN, text);}
  void error(const QString & text) {Q_EMIT logMessage(ERROR, text);}

  bool busy() const {return busy_;}
  /// Mark an operation as running (buttons disabled) or finished, then call updateButtons().
  void setBusy(bool busy);
  /// Enable/disable the buttons according to the busy state and the available data.
  virtual void updateButtons() = 0;

  /// True if an action goal sent by sendGoal() is still running.
  bool hasRunningGoal() const {return static_cast<bool>(cancel_);}
  /// Cancel the running goal: a cancel request is sent and its callback is called at once
  /// with ResultCode::CANCELED and a null result. Late results of that goal are ignored.
  void cancelRunningGoal();

  /// Run f in the GUI thread.
  void postToGui(std::function<void()> f)
  {
    QMetaObject::invokeMethod(this, std::move(f), Qt::QueuedConnection);
  }

  /// Call a service asynchronously.
  ///
  /// @param done Called in the GUI thread with the response, or with nullptr if no response
  ///             arrives within timeout_ms (the error is already logged).
  /// @return False (error logged, done not called) if the node is not initialized or the
  ///         service is not available.
  template<class SrvT>
  bool callService(
    const std::string & name, typename SrvT::Request::SharedPtr request, int timeout_ms,
    std::function<void(typename SrvT::Response::SharedPtr)> done);

  /// Send an action goal asynchronously.
  ///
  /// @param on_feedback Optional, called in the GUI thread for each feedback message.
  /// @param done Called once in the GUI thread when the goal ends. The result is null if the
  ///             goal was rejected (code UNKNOWN, error already logged) or cancelled from the GUI.
  /// @return False (error logged, done not called) if the node is not initialized, the action
  ///         server is not available or another goal of this tab is running.
  template<class ActT>
  bool sendGoal(
    const std::string & name, const typename ActT::Goal & goal,
    std::function<void(const typename ActT::Feedback &)> on_feedback,
    std::function<void(rclcpp_action::ResultCode, typename ActT::Result::SharedPtr)> done);

  /// A widget made of a toggle button with the given title and the content, hidden by default.
  static QWidget * makeCollapsible(const QString & title, QWidget * content);

  /// Join a node name and a relative interface name: ("/node/", "acquire") -> "/node/acquire".
  static std::string joinName(const QString & node, const char * name);

  rclcpp::Node::SharedPtr node_;

private:
  /// Wait at most this time for a service/action server not yet discovered (blocks the GUI).
  static constexpr std::chrono::milliseconds kDiscoveryTimeout{1000};

  template<class SrvT>
  typename rclcpp::Client<SrvT>::SharedPtr serviceClient(const std::string & name);
  template<class ActT>
  typename rclcpp_action::Client<ActT>::SharedPtr actionClient(const std::string & name);

  bool checkNode();

  std::map<std::string, rclcpp::ClientBase::SharedPtr> service_clients_;
  std::map<std::string, rclcpp_action::ClientBase::SharedPtr> action_clients_;
  std::function<void()> cancel_;  ///< cancels the running goal; empty if none
  bool busy_{false};
};

// ------------------------------------------------------------------------------------------------
// Template implementation

template<class SrvT>
typename rclcpp::Client<SrvT>::SharedPtr TabBase::serviceClient(const std::string & name)
{
  auto key = name + "|" + typeid(SrvT).name();
  auto it = service_clients_.find(key);
  if (it != service_clients_.end()) {
    return std::static_pointer_cast<rclcpp::Client<SrvT>>(it->second);
  }
  auto client = node_->create_client<SrvT>(name);
  service_clients_[key] = client;
  return client;
}

template<class ActT>
typename rclcpp_action::Client<ActT>::SharedPtr TabBase::actionClient(const std::string & name)
{
  auto key = name + "|" + typeid(ActT).name();
  auto it = action_clients_.find(key);
  if (it != action_clients_.end()) {
    return std::static_pointer_cast<rclcpp_action::Client<ActT>>(it->second);
  }
  auto client = rclcpp_action::create_client<ActT>(node_, name);
  action_clients_[key] = client;
  return client;
}

template<class SrvT>
bool TabBase::callService(
  const std::string & name, typename SrvT::Request::SharedPtr request, int timeout_ms,
  std::function<void(typename SrvT::Response::SharedPtr)> done)
{
  if (!checkNode()) {
    return false;
  }
  auto client = serviceClient<SrvT>(name);
  if (!client->service_is_ready() && !client->wait_for_service(kDiscoveryTimeout)) {
    error(QString("Service %1 not available").arg(QString::fromStdString(name)));
    return false;
  }

  // Both the response and the timeout are handled in the GUI thread: the first one wins.
  auto finished = std::make_shared<bool>(false);
  auto timer = new QTimer(this);
  timer->setSingleShot(true);

  auto pending = client->async_send_request(
    request,
    [this, finished, timer, done](typename rclcpp::Client<SrvT>::SharedFuture future) {
      auto response = future.get();
      postToGui(
        [finished, timer, done, response]() {
          if (*finished) {
            return;
          }
          *finished = true;
          timer->deleteLater();
          done(response);
        });
    });

  const int64_t request_id = pending.request_id;
  connect(
    timer, &QTimer::timeout, this,
    [this, finished, timer, client, request_id, name, timeout_ms, done]() {
      if (*finished) {
        return;
      }
      *finished = true;
      client->remove_pending_request(request_id);
      timer->deleteLater();
      error(
        QString("Service %1: no response within %2 s")
        .arg(QString::fromStdString(name)).arg(timeout_ms / 1000.0));
      done(nullptr);
    });
  timer->start(timeout_ms);
  return true;
}

template<class ActT>
bool TabBase::sendGoal(
  const std::string & name, const typename ActT::Goal & goal,
  std::function<void(const typename ActT::Feedback &)> on_feedback,
  std::function<void(rclcpp_action::ResultCode, typename ActT::Result::SharedPtr)> done)
{
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActT>;
  using ResultPtr = typename ActT::Result::SharedPtr;

  if (!checkNode()) {
    return false;
  }
  if (hasRunningGoal()) {
    error("Another action is running, cancel it first");
    return false;
  }
  auto client = actionClient<ActT>(name);
  if (!client->action_server_is_ready() && !client->wait_for_action_server(kDiscoveryTimeout)) {
    error(QString("Action %1 not available").arg(QString::fromStdString(name)));
    return false;
  }

  // State shared by the callbacks, accessed only in the GUI thread.
  auto finished = std::make_shared<bool>(false);
  auto handle = std::make_shared<typename GoalHandle::SharedPtr>();
  auto finish = [this, finished, done](rclcpp_action::ResultCode code, ResultPtr result) {
      *finished = true;
      cancel_ = nullptr;
      done(code, result);
    };

  cancel_ = [this, client, handle, finished, finish, name]() {
      if (*finished) {
        return;
      }
      if (*handle) {
        client->async_cancel_goal(*handle);
      }  // else: cancelled in goal_response_callback, as soon as the goal is accepted
      warn(QString("Cancel requested for %1").arg(QString::fromStdString(name)));
      finish(rclcpp_action::ResultCode::CANCELED, nullptr);
    };

  typename rclcpp_action::Client<ActT>::SendGoalOptions options;
  options.goal_response_callback =
    [this, client, handle, finished, finish, name](typename GoalHandle::SharedPtr goal_handle) {
      postToGui(
        [this, client, handle, finished, finish, name, goal_handle]() {
          if (!goal_handle) {
            if (!*finished) {
              error(QString("Goal rejected by %1 (see the server log)").arg(
                QString::fromStdString(name)));
              finish(rclcpp_action::ResultCode::UNKNOWN, nullptr);
            }
            return;
          }
          if (*finished) {  // cancelled before being accepted
            client->async_cancel_goal(goal_handle);
            return;
          }
          *handle = goal_handle;
        });
    };
  if (on_feedback) {
    options.feedback_callback =
      [this, finished, on_feedback](
      typename GoalHandle::SharedPtr, const std::shared_ptr<const typename ActT::Feedback> feedback) {
        postToGui(
          [finished, on_feedback, feedback]() {
            if (!*finished) {
              on_feedback(*feedback);
            }
          });
      };
  }
  options.result_callback =
    [this, finished, finish](const typename GoalHandle::WrappedResult & wrapped) {
      const auto code = wrapped.code;
      const ResultPtr result = wrapped.result;
      postToGui(
        [finished, finish, code, result]() {
          if (!*finished) {
            finish(code, result);
          }
        });
    };

  client->async_send_goal(goal, options);
  return true;
}

}  // namespace surface_gui

#endif  // SURFACE_GUI__TAB_BASE_HPP_
