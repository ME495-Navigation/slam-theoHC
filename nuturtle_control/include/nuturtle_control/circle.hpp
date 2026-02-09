#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nuturtle_control/srv/turt_circle.hpp>

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
