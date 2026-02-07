#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <catch_ros2/catch_ros2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


TEST_CASE("turtle_control node can process pure translational cmd_vel", "[turtle_control]") {

    auto node = rclcpp::Node::make_shared("integration_test_node");

    node->declare_parameter<double>("test_duration");
    const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

    auto TwistPub =
    node->create_publisher<geometry_msgs::msg::Twist>("turtle_control/cmd_vel", 10);

    bool got_msg = false;

    auto WheelCmdSub =
    node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
            "turtle_control/wheel_cmd", 10,
    [&](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
      got_msg = true;
      REQUIRE(msg->left_velocity == Catch::Approx(msg->right_velocity));
            });

    rclcpp::Time start_time = rclcpp::Clock().now();

    while(rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)))
  {
    auto twistMsg = geometry_msgs::msg::Twist();
    twistMsg.linear.x = 0.1;     // m/s
    twistMsg.angular.z = 0.0;     // rad/s

    TwistPub->publish(twistMsg);

    rclcpp::spin_some(node);
    }

    REQUIRE(got_msg);
}

TEST_CASE("turtle_control node can process pure rotational cmd_vel", "[turtle_control]") {

    auto node = rclcpp::Node::make_shared("integration_test_node");

    node->declare_parameter<double>("test_duration");
    const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

    auto TwistPub =
    node->create_publisher<geometry_msgs::msg::Twist>("turtle_control/cmd_vel", 10);

    bool got_msg = false;

    auto WheelCmdSub =
    node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
            "turtle_control/wheel_cmd", 10,
    [&](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
      got_msg = true;
      REQUIRE(-1.0 * msg->left_velocity == Catch::Approx(msg->right_velocity));
            });

    rclcpp::Time start_time = rclcpp::Clock().now();

    while(rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)))
  {
    auto twistMsg = geometry_msgs::msg::Twist();
    twistMsg.linear.x = 0.0;     // m/s
    twistMsg.angular.z = 0.1;     // rad/s

    TwistPub->publish(twistMsg);

    rclcpp::spin_some(node);
    }

    REQUIRE(got_msg);
}

TEST_CASE("encoder data in sensor_msg is properly converted to join states", "[turtle_control]") {

  auto node = rclcpp::Node::make_shared("integration_test_node");

  node->declare_parameter<double>("test_duration");
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  node->declare_parameter<double>("encoder_ticks_per_rad");
  const auto TICKS_PER_RAD =
    node->get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();

  auto SensorDataPub =
    node->create_publisher<nuturtlebot_msgs::msg::SensorData>("turtle_control/sensor_data", 10);

  bool got_msg = false;

  auto JointStateSub =
    node->create_subscription<sensor_msgs::msg::JointState>(
          "turtle_control/joint_states", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
      got_msg = true;
      REQUIRE(msg->position.size() == 2);
      REQUIRE(msg->position[0] == Catch::Approx(1.0 / TICKS_PER_RAD));
      REQUIRE(msg->position[1] == Catch::Approx(1.0 / TICKS_PER_RAD));
          });

  rclcpp::Time start_time = rclcpp::Clock().now();
  while(rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    !got_msg)
  {
    auto sensorDataMsg = nuturtlebot_msgs::msg::SensorData();
    sensorDataMsg.left_encoder = 1;
    sensorDataMsg.right_encoder = 1;

    SensorDataPub->publish(sensorDataMsg);
    rclcpp::spin_some(node);
    }
}
