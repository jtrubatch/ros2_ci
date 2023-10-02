#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tortoisebot_action/action/waypoints.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <thread>
#define PI 3.1415926

using std::placeholders::_1;
using std::placeholders::_2;

class WaypointActionServer : public rclcpp::Node {
public:
  using Waypoint = tortoisebot_action::action::Waypoints;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Waypoint>;

  explicit WaypointActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("tortoisebot_as", options)
    {
        this->action_server_ = rclcpp_action::create_server<Waypoint>(
        this, "tortoisebot",
        std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
        std::bind(&WaypointActionServer::handle_cancel, this, _1),
        std::bind(&WaypointActionServer::handle_accepted, this, _1));
    
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&WaypointActionServer::odom_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Action Server Initiated"); 
  }

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Point goal_position;
  geometry_msgs::msg::Twist cmd;
  float yaw;
  float yaw_error;
  float position_error;
  float distance_precision = 0.05;
  float yaw_precision = PI / 90;
  tf2::Quaternion qyaw;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position = msg->pose.pose.position;
    tf2::convert(msg->pose.pose.orientation, qyaw);
    yaw = tf2::getYaw(qyaw);
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                    std::shared_ptr<const Waypoint::Goal> goal) 
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request X: %f Y: %f", goal->position.x, goal->position.y);  
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;          
  }
  
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) 
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) 
  {
    std::thread{std::bind(&WaypointActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {

    const auto goal = goal_handle->get_goal();
    bool accepted = true;
    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto result = std::make_shared<Waypoint::Result>();
    rclcpp::Rate loop_rate(25);

    goal_position = goal->position;
    float goal_yaw = atan2(goal_position.y - position.y, goal_position.x - position.x);  
    while (accepted) {
      position_error = sqrt(pow(goal_position.y - position.y, 2) +
                            pow(goal_position.x - position.x, 2));
                          
      yaw_error = goal_yaw - yaw;

      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        feedback->state = "CANCELLED";
        goal_handle->publish_feedback(feedback);
        accepted = false;
        rclcpp::shutdown();
      }
      if (abs(yaw_error) > yaw_precision) {
        if (yaw_error > 0) {
          cmd.angular.z = 0.65;
        } else {
          cmd.angular.z = -0.65;
        }
        cmd_pub_->publish(cmd);
      } else {
        cmd.linear.x = 0.4;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
      }
      feedback->position = position;
      feedback->state = "EXECUTING";
      goal_handle->publish_feedback(feedback);
      //RCLCPP_INFO(this->get_logger(), "Position Error: %f", position_error);
      if (position_error < distance_precision) {
        accepted = false;
        cmd.angular.z = 0.0;
        cmd.linear.x = 0.0;
        cmd_pub_->publish(cmd);
        result->success = true;
        feedback->state = "COMPLETE";
        goal_handle->publish_feedback(feedback);
        goal_handle->succeed(result);
      }
      loop_rate.sleep();

    } // END WHILE
  }   // END EXECUTE

}; // END CLASS

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto server = std::make_shared<WaypointActionServer>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(server);
    exec.spin();

    rclcpp::shutdown();

    return 0;
}