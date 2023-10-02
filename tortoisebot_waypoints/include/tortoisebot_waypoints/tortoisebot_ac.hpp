#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tortoisebot_action/action/waypoints.hpp"
#include <functional>
#include <memory>

class WaypointClient : public rclcpp::Node {
public:
  using Waypoint = tortoisebot_action::action::Waypoints;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Waypoint>;

  explicit WaypointClient(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void send_goal();
  

private:
  rclcpp_action::Client<Waypoint>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Point goal;
  void response_callback(const GoalHandle::SharedPtr &goal_handle);
  void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const Waypoint::Feedback> feedback);
  void result_callback(const GoalHandle::WrappedResult &result);

}; // END CLASS
