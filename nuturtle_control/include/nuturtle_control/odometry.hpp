#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <turtlelib/diff_drive.hpp>

class odometry : public rclcpp::Node{
    public:
        odometry();

    private:
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jsSub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        // //service called initposeserv taking a transformstamped request and empty response
        // rclcpp::Service<>::SharedPtr initPoseServ;

        void odomCallback(const sensor_msgs::msg::JointState msg);

        turtlelib::DiffDrive robotState;

        bool hasSetWheels = false;
};
