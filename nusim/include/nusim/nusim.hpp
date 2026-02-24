#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <turtlelib/diff_drive.hpp>

using namespace std::chrono_literals;

/// \brief A simulator node for the nuturtle project
class nusimulator : public rclcpp::Node{
public:
                /// \brief Construct a nusimulator node
  nusimulator();

private:
  rclcpp::TimerBase::SharedPtr simtick;
  rclcpp::TimerBase::SharedPtr fake_sensor_tick;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timesteppub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resetsrv;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wallpub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstaclepub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_obstaclepub;

  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensordatapub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheelcmdsub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointpub;

  turtlelib::DiffDrive robotState;

  double left_wheel_vel;
  double right_wheel_vel;

  int count;

  const float wall_thickness = .1;

                /// \brief Timer callback to publish timestep and transform
  void sim_tick_callback();

                /// \brief Check for collisions and update robot pose if collision occurs
  void collision_check();

  void wheelcmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg);

                /// \brief Reset callback to reset robot state
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

                /// \brief Publish obstacle markers
  void publish_obstacle();

                /// \brief Publish wall markers
  void publish_real_walls();

                /// \brief Generate a wall pair marker
                /// \param side - if true, generate vertical walls; if false, generate horizontal walls
                /// \return a Marker representing the wall pair
  visualization_msgs::msg::Marker genWallPair(bool side);

  // ##### begin_citation [23] #####
                /// \brief Create an obstacle marker with specified properties
                /// \param x - x position of obstacle
                /// \param y - y position of obstacle
                /// \param color - color of the obstacle marker
                /// \param action - action type for the marker (0 = add/modify, 1 = delete, etc.)
                /// \return a Marker representing the obstacle
  visualization_msgs::msg::Marker create_obstacle_marker(double x, double y, 
    std_msgs::msg::ColorRGBA color, int action, std::string frame_id, std::string ns = "");
  // ##### end_citation [23] #####

  void fake_sensor_tick_callback();
};
