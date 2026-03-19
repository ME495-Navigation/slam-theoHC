#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <turtlelib/diff_drive.hpp>

using namespace std::chrono_literals;

/// @brief Kinematics simulator for the NUturtle differential-drive robot.
///
/// Simulates robot kinematics, wheel slip, input noise, cylindrical obstacle collisions,
/// a fake proximity sensor, and a ray-cast LIDAR. All simulated topics are namespaced
/// under `red/`.
///
/// @par Subscribed Topics
/// - `red/wheel_cmd` (nuturtlebot_msgs/WheelCommands): Motor velocity commands to simulate.
///
/// @par Published Topics
/// - `~/timestep` (std_msgs/UInt64): Monotonically increasing simulation tick counter.
/// - `~/real_walls` (visualization_msgs/MarkerArray): Arena boundary walls for visualisation.
/// - `~/real_obstacles` (visualization_msgs/MarkerArray): Ground-truth obstacle positions.
/// - `~/fake_sensor` (visualization_msgs/MarkerArray): Noisy landmark detections visible to
///   the SLAM node (obstacles within `basic_sensor_range`).
/// - `red/sensor_data` (nuturtlebot_msgs/SensorData): Simulated encoder tick counts,
///   optionally corrupted by slip and input noise.
/// - `red/joint_states` (sensor_msgs/JointState): Simulated wheel joint positions [rad].
/// - `red/path` (nav_msgs/Path): Ground-truth path history of the simulated robot.
/// - `red/lidar` (sensor_msgs/LaserScan): Ray-cast LIDAR scan with optional noise.
///
/// @par Broadcast TF Frames
/// - `nusim/world` → `red/base_footprint`: Ground-truth pose of the simulated robot.
///
/// @par Services
/// - `~/reset` (std_srvs/Empty): Resets the robot to its initial pose (x0, y0, theta0) and
///   clears the path history.
///
/// @par Parameters
/// - `rate` (int): Simulation update rate [ms per tick].
/// - `x0` (double, default `0.0`): Initial robot x position [m].
/// - `y0` (double, default `0.0`): Initial robot y position [m].
/// - `theta0` (double, default `0.0`): Initial robot heading [rad].
/// - `arena_x_length` (double, default `5.0`): Arena width [m].
/// - `arena_y_length` (double, default `5.0`): Arena height [m].
/// - `obstacles.x` (double[], default `[]`): X positions of cylindrical obstacles [m].
/// - `obstacles.y` (double[], default `[]`): Y positions of cylindrical obstacles [m].
/// - `obstacles.r` (double, default `3.0`): Radius of each obstacle [m].
/// - `lidar_min_range` (double, default `0.12`): Minimum LIDAR range [m].
/// - `lidar_max_range` (double, default `3.5`): Maximum LIDAR range [m].
/// - `lidar_num_points` (int, default `360`): Number of rays per LIDAR scan.
/// - `lidar_noise_stddev` (double, default `0.0`): Std dev of Gaussian noise on LIDAR ranges [m].
/// - `wheel_radius` (double, **required**): Wheel radius [m].
/// - `track_width` (double, **required**): Wheel-to-wheel distance [m].
/// - `encoder_ticks_per_rad` (double, **required**): Encoder ticks per radian.
/// - `motor_cmd_max` (double, **required**): Maximum motor command magnitude.
/// - `motor_cmd_per_rad_sec` (double, **required**): Motor command units per rad/s.
/// - `collision_radius` (double, default `0.08`): Robot collision radius for obstacle detection [m].
/// - `basic_sensor_range` (float, default `5.0`): Maximum range of the fake proximity sensor [m].
/// - `input_noise` (float, default `0.0`): Variance of Gaussian noise added to wheel commands.
/// - `slip_fraction` (float, default `0.0`): Fraction of random wheel slip applied each tick.
/// - `basic_sensor_variance` (float, default `0.0`): Variance of noise on fake sensor detections.
/// - `lidar_variance` (float, default `0.0`): Variance of noise on LIDAR ranges.
/// - `real_bot_path_length` (int, default `1000`): Maximum number of poses retained in the path.
class nusimulator : public rclcpp::Node{
public:
  /// @brief Construct and initialise the nusimulator node.
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
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathpub;

  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensordatapub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheelcmdsub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointpub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscanpub;

  turtlelib::DiffDrive robotState;

  double left_wheel_vel;
  double right_wheel_vel;
  turtlelib::Vector2D reported_wheel_positions; //the wheel positions that we report to the user through the sensor data topic. This may differ from the actual wheel positions due to noise.

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

  std::deque<geometry_msgs::msg::PoseStamped> real_bot_path;
};
