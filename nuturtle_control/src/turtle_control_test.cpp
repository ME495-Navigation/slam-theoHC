#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <catch_ros2/catch_ros2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>


TEST_CASE("turtle_control node can process pure translational cmd_vel", "[turtle_control]"){

    auto node = rclcpp::Node::make_shared("integration_test_node");

    node->declare_parameter<double>("test_duration");
    const auto TEST_DURATION =
        node->get_parameter("test_duration").get_parameter_value().get<double>();

    auto TwistPub =
        node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

    bool got_msg = false;

    auto WheelCmdSub =
        node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
            "turtle1/wheel_cmd", 10,
            [&](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) { 
                got_msg = true;  
                REQUIRE(msg->left_velocity == Catch::Approx(1.0));
                REQUIRE(msg->right_velocity == Catch::Approx(1.0));
            });

    rclcpp::Time start_time = rclcpp::Clock().now();

    while(rclcpp::ok() && ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))){
        auto twistMsg = geometry_msgs::msg::Twist();
        twistMsg.linear.x = 0.1; // m/s
        twistMsg.angular.z = 0.0; // rad/s

        TwistPub->publish(twistMsg);

        rclcpp::spin_some(node);
    }

    REQUIRE(got_msg);
}