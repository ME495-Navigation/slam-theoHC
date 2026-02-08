#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <catch_ros2/catch_ros2.hpp>
#include <nuturtle_control/srv/odom_config.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/joint_state.hpp>

TEST_CASE("Test initial pose service of the odometry node", "[odometry]") {
    auto node = rclcpp::Node::make_shared("turtle_odom_test_node");

    node->declare_parameter<double>("test_duration");
    const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

    auto initPoseClient =
    node->create_client<nuturtle_control::srv::OdomConfig>("/odometry/initial_pose");

    rclcpp::Time start_time = rclcpp::Clock().now();
    bool got_response = false;
    while(rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    !got_response)
  {
    if (!initPoseClient->wait_for_service(std::chrono::seconds(1))) {
      continue;
    }

    auto request = std::make_shared<nuturtle_control::srv::OdomConfig::Request>();
    request->x = 1.0;
    request->y = 2.0;
    request->theta = 3.14;
    auto result_future = initPoseClient->async_send_request(request);
        // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result_future.get();
      got_response = true;
      break;
    } else {
      REQUIRE(false);    // fail the test if the service call failed
    }
    rclcpp::spin_some(node);
    }
    REQUIRE(got_response);
}

TEST_CASE("Test that transform between base_footprint and odom exists", "[odometry]") {
    auto node = rclcpp::Node::make_shared("turtle_odom_test_node");

    node->declare_parameter<double>("test_duration");
    const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // publish a joint state message to trigger the odometry node to publish a transform
    auto JointStatePub =
    node->create_publisher<sensor_msgs::msg::JointState>("/odometry/joint_states", 10);

    rclcpp::Time start_time = rclcpp::Clock().now();
    bool got_transform = false;
    while(rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    !got_transform)
  {
    auto jointStateMsg = sensor_msgs::msg::JointState();
    jointStateMsg.name = {"left_wheel_joint", "right_wheel_joint"};
    jointStateMsg.position = {0.0, 0.0};
    jointStateMsg.header.stamp = node->get_clock()->now();
    JointStatePub->publish(jointStateMsg);

    try {
      auto transform = tf_buffer.lookupTransform(
                "odom", "base_footprint", tf2::TimePointZero);
      got_transform = true;
      break;
    } catch (const tf2::TransformException & ex) {
            // If the transform is not available yet, just continue and try again.
      continue;
    }
    rclcpp::spin_some(node);
    }
    REQUIRE(got_transform);
}
