#include "tortoisebot_waypoints/tortoisebot_ac.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tortoisebot_action/action/waypoints.hpp"
#include <functional>
#include <future>
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;
using Waypoint = tortoisebot_action::action::Waypoints;
using GoalHandle = rclcpp_action::ClientGoalHandle<Waypoint>;

WaypointClient::WaypointClient(const rclcpp::NodeOptions &options)
    : Node("waypoint_action_client", options) {

  this->client_ = rclcpp_action::create_client<Waypoint>(this, "tortoisebot");
  this->timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&WaypointClient::send_goal, this));
  this->goal.x = 1.25;
  this->goal.y = 2.25;
  RCLCPP_INFO(this->get_logger(), "Client Initiated");
}

void WaypointClient::send_goal() {
  this->timer_->cancel();
  if (!this->client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
  }

  auto goal_msg = Waypoint::Goal();
  goal_msg.position = this->goal;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto goal_options = rclcpp_action::Client<Waypoint>::SendGoalOptions();
  goal_options.goal_response_callback =
      std::bind(&WaypointClient::response_callback, this, _1);
  goal_options.feedback_callback =
      std::bind(&WaypointClient::feedback_callback, this, _1, _2);
  goal_options.result_callback =
      std::bind(&WaypointClient::result_callback, this, _1);

  this->client_->async_send_goal(goal_msg, goal_options);
}

void WaypointClient::response_callback(
    const GoalHandle::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void WaypointClient::feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const Waypoint::Feedback> feedback) {
  // RCLCPP_INFO(this->get_logger(), "State is: %s", feedback->state.c_str());
}

void WaypointClient::result_callback(const GoalHandle::WrappedResult &result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
}
