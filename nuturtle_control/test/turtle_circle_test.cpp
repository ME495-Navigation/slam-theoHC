#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <catch_ros2/catch_ros2.hpp>
#include <geometry_msgs/msg/twist.hpp>

// ########### begin_citation [16] ###########
TEST_CASE("circle node publishes cmd_vel at 100hz", "[circle]") {
  auto node = rclcpp::Node::make_shared("turtle_circle_test_node");

  node->declare_parameter<double>("test_duration");
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  int numreceived = 0;

  auto TwistSub =
    node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
      [&](const geometry_msgs::msg::Twist::SharedPtr) {
        numreceived++;
    });

  rclcpp::Time start_time = rclcpp::Clock().now();

  while(rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)))
  {
    rclcpp::spin_some(node);
  }

  REQUIRE(numreceived / TEST_DURATION == Catch::Approx(100.0).margin(10.0));
}
// ########### end_citation [16] ###########
