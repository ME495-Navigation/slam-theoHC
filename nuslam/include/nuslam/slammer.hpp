#pragma once
#include "rclcpp/rclcpp.hpp"
#include "turtlelib/ekf.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <vector>

/// @brief Extended Kalman Filter SLAM node for a differential-drive robot.
///
/// Fuses odometry (for the EKF prediction step) with landmark detections from a proximity
/// sensor (for the EKF update step). Maintains a joint state estimate over the robot pose
/// and the positions of all detected cylindrical landmarks. New landmarks are initialised
/// on first observation and the state vector is extended dynamically.
///
/// @par Subscribed Topics
/// - `odom` (nav_msgs/Odometry): Odometric pose estimate used as the EKF process input.
/// - `fake_input` (visualization_msgs/MarkerArray): Landmark detections (cylindrical obstacles)
///   from the simulator's fake proximity sensor, used for the EKF measurement update.
///
/// @par Published Topics
/// - `slam_landmarks` (visualization_msgs/MarkerArray): EKF-estimated landmark positions in the
///   map frame, visualised as cylinders.
///
/// @par Broadcast TF Frames
/// - `map_id` → `odom_id`: Correction transform computed by the EKF that aligns the odometry
///   frame with the globally consistent map frame.
///
/// @par Parameters
/// - `obstacle_radius` (double, default `0.5`): Radius used when rendering landmark markers [m].
/// - `map_id` (string, default `"map"`): Name of the global map frame.
/// - `odom_id` (string, default `"odom"`): Name of the odometry frame.
class Slammer : public rclcpp::Node {
    public:
        Slammer();
    
    private:
        turtlelib::DiffDriveEKF ekf;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fakeInputSub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slamMarkerPub;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void fakeInputCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

        int numLandmarks = 0;

        std::vector<int> LandmarkIDtoIndex;
};