#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nuslam/ekf.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class Slammer : public rclcpp::Node {
    public:
        Slammer();
    
    private:
        ExtendedKalmanFilter ekf;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fakeInputSub;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void fakeInputCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
};