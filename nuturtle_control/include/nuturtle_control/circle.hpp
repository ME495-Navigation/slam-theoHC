#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nuturtle_control/srv/turt_circle.hpp>

/// @brief Commands a robot to drive in a circle at a configurable radius and angular velocity.
///
/// Publishes Twist commands on a fixed-rate timer. The robot starts stationary; motion is
/// toggled via the `stop` service and direction is reversed via the `reverse` service.
///
/// @par Published Topics
/// - `cmd_vel` (geometry_msgs/Twist): Velocity command computed from the current radius and
///   angular velocity. Published at `frequency` Hz. Zero twist is published while stopped.
///
/// @par Services
/// - `stop` (std_srvs/Empty): Toggles motion on/off. The robot starts in the stopped state.
/// - `reverse` (std_srvs/Empty): Flips the direction of travel (clockwise ↔ counter-clockwise).
/// - `circle` (nuturtle_control/TurtCircle): Sets the angular velocity [rad/s] and radius [m]
///   of the circular trajectory.
///
/// @par Parameters
/// - `frequency` (int, default `100`): Timer frequency [Hz] at which `cmd_vel` is published.
class circle : public rclcpp::Node{
public:
  circle();

private:
  float angularVel = 1;
  float radius = 0.15;

  bool moving = false;
  bool clockwise = true;
  bool forward = false;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr startServ;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverseServ;
  rclcpp::Service<nuturtle_control::srv::TurtCircle>::SharedPtr circleServ;


  rclcpp::TimerBase::SharedPtr controlTimer;

  void controlCallback();
  void startCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void reverseCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void circleCallback(
    const std::shared_ptr<nuturtle_control::srv::TurtCircle::Request> request,
    std::shared_ptr<nuturtle_control::srv::TurtCircle::Response> response);

};
