#pragma once

// ######### begin_citation [30] #########
#include <deque>
// ######### end_citation [30] #########
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
// ######### begin_citation [30] #########
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// ######### end_citation [30] #########
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <turtlelib/diff_drive.hpp>
#include <nuturtle_control/srv/odom_config.hpp>

class odometry : public rclcpp::Node{
public:
  odometry();

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jsSub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
  // ######### begin_citation [30] #########
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr uncorrectedPathPub;
  // ######### end_citation [30] #########
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Service<nuturtle_control::srv::OdomConfig>::SharedPtr initPoseServ;
  // ######### begin_citation [30] #########
  std::deque<geometry_msgs::msg::PoseStamped> odom_path;
  std::deque<geometry_msgs::msg::PoseStamped> uncorrected_path;
  // ######### end_citation [30] #########

  void odomCallback(const sensor_msgs::msg::JointState msg);

  void initPoseCallback(
    const std::shared_ptr<nuturtle_control::srv::OdomConfig::Request> request,
    std::shared_ptr<nuturtle_control::srv::OdomConfig::Response> response);

  turtlelib::DiffDrive robotState;

  bool hasSetWheels = false;
};
