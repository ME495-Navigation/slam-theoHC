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

/// @brief Computes wheel odometry from joint states and broadcasts the robot's pose.
///
/// Integrates wheel positions from `joint_states` using differential-drive kinematics to
/// maintain an odometric estimate of the robot pose. Optionally tracks a second, uncorrected
/// odometry frame so that SLAM-corrected and raw odometry can be visualised simultaneously.
///
/// @par Subscribed Topics
/// - `joint_states` (sensor_msgs/JointState): Wheel joint positions [rad] used to compute
///   forward kinematics.
///
/// @par Published Topics
/// - `odom` (nav_msgs/Odometry): Current odometric pose and velocity estimate.
/// - `odom_path` (nav_msgs/Path): Full pose history of the (possibly SLAM-corrected) odometry.
/// - `uncorrected_odom_path` (nav_msgs/Path): Pose history of the raw, uncorrected odometry
///   (only meaningful when an `uncorrected_body_id` frame is configured).
///
/// @par Broadcast TF Frames
/// - `odom_id` → `body_id`: Main odometry-to-body transform.
/// - `uncorrected_odom_id` → `uncorrected_body_id` *(optional)*: Raw odometry transform when
///   both uncorrected frame parameters are set.
///
/// @par Services
/// - `initial_pose` (nuturtle_control/OdomConfig): Teleports the odometry origin; resets the
///   robot pose to the requested (x, y, theta) without moving the physical robot.
///
/// @par Parameters
/// - `body_id` (string, default `"base_footprint"`): Robot body frame name.
/// - `odom_id` (string, default `"odom"`): Odometry frame name.
/// - `uncorrected_body_id` (string, *optional*): Secondary body frame for raw odometry.
/// - `uncorrected_odom_id` (string, *optional*): Secondary odom frame for raw odometry.
/// - `wheel_left` (string, **required**): Name of the left wheel joint in `joint_states`.
/// - `wheel_right` (string, **required**): Name of the right wheel joint in `joint_states`.
/// - `wheel_radius` (double, **required**): Radius of the drive wheels [m].
/// - `track_width` (double, **required**): Distance between the two wheel centres [m].
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
