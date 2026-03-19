#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <turtlelib/diff_drive.hpp>

/// @brief Translates between high-level velocity commands and low-level motor/sensor interfaces.
///
/// @par Subscribed Topics
/// - `cmd_vel` (geometry_msgs/Twist): Body-frame velocity command. The y component is ignored
///   (non-holonomic constraint); a warning is logged if it is non-zero.
/// - `sensor_data` (nuturtlebot_msgs/SensorData): Raw encoder tick counts from the robot hardware.
///
/// @par Published Topics
/// - `wheel_cmd` (nuturtlebot_msgs/WheelCommands): Integer motor velocity commands sent to
///   the robot's motor controllers.
/// - `joint_states` (sensor_msgs/JointState): Wheel joint positions (rad) derived from encoder
///   ticks, intended for odometry and visualisation consumers.
///
/// @par Parameters
/// - `wheel_radius` (double, **required**): Radius of the drive wheels [m].
/// - `track_width` (double, **required**): Distance between the two wheel centres [m].
/// - `encoder_ticks_per_rad` (double, **required**): Encoder ticks per radian of wheel travel.
/// - `motor_cmd_per_rad_sec` (double, **required**): Motor command units per rad/s of wheel speed.
class turtle_control : public rclcpp::Node{
public:
  turtle_control();

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensorDataSub;

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void sensorDataCallback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg);

  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheelCommander;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub;

  turtlelib::DiffDrive robotState;
};
