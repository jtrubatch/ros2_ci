#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include "tortoisebot_action/action/waypoints.hpp"
#include "tortoisebot_waypoints/tortoisebot_ac.hpp"
#include "gtest/gtest.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;
using Waypoint = tortoisebot_action::action::Waypoints;
class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
}; // END RCLCPPFIXTURE CLASS
RclCppFixture g_rclcppfixture;

class WaypointTestFixture : public ::testing::Test {
public:
  WaypointTestFixture() {
    AcNode = std::make_shared<WaypointClient>();
    TestNode = rclcpp::Node::make_shared("test_node");
    test_sub_ = TestNode->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&WaypointTestFixture::odom_callback, this, _1));
    first_call = true;
    goal_reached = false;
  }

  float getGoalYaw();
  bool PositionTest();
  bool OrientationTest();

protected:
  std::shared_ptr<rclcpp::Node> AcNode;
  std::shared_ptr<rclcpp::Node> TestNode;

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position = msg->pose.pose.position;
    tf2::convert(msg->pose.pose.orientation, current_orientation);
    yaw = tf2::getYaw(current_orientation);
    if (first_call) {
      initial_position = current_position;
      first_call = false;
    }
    heartbeat = TestNode->count_publishers("/cmd_vel");
  }
  geometry_msgs::msg::Point initial_position;
  geometry_msgs::msg::Point current_position;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr test_sub_;
  tf2::Quaternion current_orientation;
  float yaw;
  float goal_yaw;
  bool first_call;
  bool goal_reached;
  float goal_x = 1.25;
  float goal_y = 2.25;
  size_t heartbeat = 1;

}; // END TEST CLASS
float WaypointTestFixture::getGoalYaw() {
  return atan2(goal_y - this->initial_position.y,
               goal_x - this->initial_position.x);
}

bool WaypointTestFixture::PositionTest() {
  float position_margin = 0.1;
  float x_error;
  float y_error;
  while (heartbeat) {
    rclcpp::spin_some(AcNode);
    rclcpp::spin_some(TestNode);
    x_error = abs(goal_x - this->current_position.x);
    y_error = abs(goal_y - this->current_position.y);
    if (x_error <= position_margin && y_error <= position_margin) {
      goal_reached = true;
      break;
    }
  }
  if (goal_reached) {
    return true;
  } else if (!heartbeat) {
    return false;
  }
}

bool WaypointTestFixture::OrientationTest() {
  float yaw_margin = 5.0;
  float yaw_error;
  bool yaw_reached = false;
  while (heartbeat) {
    yaw_error = abs(this->getGoalYaw() - this->yaw);
    if (yaw_error <= yaw_margin) {
      yaw_reached = true;
      break;
    }
  }
  if (yaw_reached) {
    return true;
  } else if (!heartbeat) {
    return false;
  }
}

TEST_F(WaypointTestFixture, Position) { EXPECT_TRUE(PositionTest()); }

TEST_F(WaypointTestFixture, Orientation) { EXPECT_TRUE(OrientationTest()); }
// COMMENT FOR TEST CHANGED